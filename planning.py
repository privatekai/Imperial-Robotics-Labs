import math, time

import cv2
from picamera2 import Picamera2
from picameracans import captureCanCentroids, displayImg, FONT
from picamerahomographygrid import drawGridOnImage, HtransformUVtoXY, HInv

from constants import (
    ANGLE_ERROR, BP, BP_SENSOR_ERROR, LEFT_MOTOR_PORT as LEFT_PORT, PI, POSITION_TOLERANCE, RIGHT_MOTOR_PORT as RIGHT_PORT,
    LEFT_TOUCH_PORT, RIGHT_TOUCH_PORT, TIMEOUT, TURNING_SPEED, WHEEL_CIRCUMFERENCE,
    WHEEL_CIRCUMFERENCE_CM, WHEELBASE_CM,
    ROBOTRADIUS, B_RADIUS, SAFEDIST,
    MAXVELOCITY, MAXACCELERATION,
    GOAL_TOLERANCE, MAX_BARRIERS,
    FORWARDWEIGHT, OBSTACLEWEIGHT, SPEEDWEIGHT, HEADINGWEIGHT, TAU, WHEELBASE_WIDTH,
    WHITE, Y_UNCERTAINTY, X_UNCERTAINTY, CAM_DIST
)

# Start camera
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(preview_config)
picam2.start()

# Timestep delta to run control at
dt = 0.4

# Target location (cm) — 4.5m ahead along y-axis
target = (0, 450)

# Barrier (obstacle) locations — list of (x, y) tuples in cm
# Populated at runtime by camera detection pipeline
barriers = []


def cm_per_sec_to_dps(v_cm):
    """Convert velocity in cm/s to motor degrees per second."""
    return (v_cm / WHEEL_CIRCUMFERENCE_CM) * 360.0


def encoder_deg_to_cm(deg):
    """Convert encoder degrees to distance in cm."""
    return (deg / 360.0) * WHEEL_CIRCUMFERENCE_CM


def predictPosition(vL, vR, x, y, theta, deltat):
    """Predict new robot position based on current pose and velocity controls.
    All distances in cm, theta in radians."""
    if abs(vL - vR) < 1e-6:
        xnew = x + vL * deltat * math.cos(theta)
        ynew = y + vL * deltat * math.sin(theta)
        thetanew = theta
    elif abs(vL + vR) < 1e-6:
        xnew = x
        ynew = y
        thetanew = theta + ((vR - vL) * deltat / WHEELBASE_CM)
    else:
        R = WHEELBASE_CM / 2.0 * (vR + vL) / (vR - vL)
        deltatheta = (vR - vL) * deltat / WHEELBASE_CM
        xnew = x + R * (math.sin(deltatheta + theta) - math.sin(theta))
        ynew = y - R * (math.cos(deltatheta + theta) - math.cos(theta))
        thetanew = theta + deltatheta

    return (xnew, ynew, thetanew)


def add_barrier(world_x, world_y):
    """Add a barrier if not a duplicate of an existing one."""
    for (bx, by) in barriers:
        if abs(bx - world_x) < B_RADIUS + X_UNCERTAINTY and abs(by - world_y) < B_RADIUS + Y_UNCERTAINTY:
            return False
    barriers.append((world_x, world_y))
    return True


def calculateClosestObstacleDistance(x, y):
    """Calculate distance to the closest obstacle from position (x, y). All in cm."""
    closestdist = float('inf')
    for barrier in barriers:
        dx = barrier[0] - x
        dy = barrier[1] - y
        d = math.sqrt(dx**2 + dy**2)
        dist = d - B_RADIUS - ROBOTRADIUS
        if dist < closestdist:
            closestdist = dist
    return closestdist


def dwa_choose_velocities(x, y, theta, vL, vR, verbose=False):
    """Run DWA and return (vL_chosen, vR_chosen, debug_info_dict).

    If *verbose* is True, print a decision summary to stdout.
    """
    bestBenefit = -float('inf')
    vLchosen = vL
    vRchosen = vR

    steps = [i * MAXACCELERATION * dt for i in range(-4, 5)]
    vLpossiblearray = list(set([vL + s for s in steps if vL + s != 0]))
    vRpossiblearray = list(set([vR + s for s in steps if vR + s != 0]))

    goal_heading = math.atan2(target[1] - y, target[0] - x)

    candidates_evaluated = 0
    candidates_clamped = 0
    best_forward = 0.0
    best_heading = 0.0
    best_obs_cost = 0.0
    best_speed_cost = 0.0
    best_obs_dist = float('inf')

    type = "a"
    if x==0.0 and y==0.0:
        type = "w"
    with open("barrier_out.txt", type) as f:
        f.write("--- BARRIERS --- \n")
        for barrier in barriers:
            f.write(str(barrier) + "\n")
        f.write("--- ROBOT POS --- \n")
        f.write("pos: " + str(x) + ", " + str(y) + ", " + str(theta) + "\n")
        f.close()

    for vLpossible in vLpossiblearray:
        for vRpossible in vRpossiblearray:
            if abs(vLpossible) <= MAXVELOCITY and abs(vRpossible) <= MAXVELOCITY:
                candidates_evaluated += 1
                (xpredict, ypredict, thetapredict) = predictPosition(
                    vLpossible, vRpossible, x, y, theta, TAU)

                # Check obstacle distance along entire trajectory, not just endpoint
                min_obstacle_dist = float('inf')
                for s in range(1, 6):
                    t = TAU * s / 5
                    xp, yp, _ = predictPosition(vLpossible, vRpossible, x, y, theta, t)
                    d = calculateClosestObstacleDistance(xp, yp)
                    if d < min_obstacle_dist:
                        min_obstacle_dist = d
                distanceToObstacle = min_obstacle_dist

                previousTargetDistance = math.sqrt((x - target[0])**2 + (y - target[1])**2)
                newTargetDistance = math.sqrt((xpredict - target[0])**2 + (ypredict - target[1])**2)
                distanceForward = previousTargetDistance - newTargetDistance

                distanceBenefit = FORWARDWEIGHT * distanceForward

                if distanceToObstacle < SAFEDIST:
                    obstacleCost = OBSTACLEWEIGHT * (SAFEDIST - distanceToObstacle)
                    speed = (abs(vLpossible) + abs(vRpossible)) / 2.0
                    speedCost = SPEEDWEIGHT * speed * (SAFEDIST - distanceToObstacle) / SAFEDIST
                else:
                    obstacleCost = 0.0
                    speedCost = 0.0

                # Project displacement onto the vector from robot to target,
                # normalised by the max distance the robot can travel in TAU.
                to_target_x = target[0] - x
                to_target_y = target[1] - y
                to_target_dist = math.sqrt(to_target_x**2 + to_target_y**2)
                dx_moved = xpredict - x
                dy_moved = ypredict - y
                if to_target_dist > 0 and MAXVELOCITY * TAU > 0:
                    projection = (dx_moved * to_target_x + dy_moved * to_target_y) / to_target_dist
                    headingBenefit = HEADINGWEIGHT * projection / (MAXVELOCITY * TAU)
                else:
                    headingBenefit = 0.0

                with open("planning_out.txt", type) as f:
                    f.write("--- CANDIDATE --- \n")
                    f.write("vL: " + str(vLpossible) + "\n")
                    f.write("vR: " + str(vRpossible) + "\n")
                    f.write("distance benefit: " + str(distanceBenefit) + "\n")
                    f.write("movement-to-target benefit: " + str(headingBenefit) + "\n")
                    f.write("obstacle cost: " + str(obstacleCost) + "\n")
                    f.write("speed cost: " + str(speedCost) + "\n")

                    f.close()

                benefit = distanceBenefit - obstacleCost - speedCost + headingBenefit
                if benefit > bestBenefit:
                    vLchosen = vLpossible
                    vRchosen = vRpossible
                    bestBenefit = benefit
                    best_forward = distanceBenefit
                    best_heading = headingBenefit
                    best_obs_cost = obstacleCost
                    best_speed_cost = speedCost
                    best_obs_dist = distanceToObstacle
            else:
                candidates_clamped += 1

    debug = {
        'candidates_evaluated': candidates_evaluated,
        'candidates_clamped': candidates_clamped,
        'bestBenefit': bestBenefit,
        'best_forward': best_forward,
        'best_heading': best_heading,
        'best_obs_cost': best_obs_cost,
        'best_speed_cost': best_speed_cost,
        'best_obs_dist': best_obs_dist,
    }

    with open("barrier_out.txt", "a") as f:
        f.write("--- CHOSEN --- \n")
        f.write("vL: " + str(vLchosen) + ", vR: " + str(vRchosen) + "\n")
        f.write("bestForward: " + str(best_forward) + ", bestObstacleCost: " + str(best_obs_cost) + "\n")
        f.close()

    if verbose:
        dist_to_target = math.sqrt((x - target[0])**2 + (y - target[1])**2)
        print("--- DWA ---")
        print("  pose: (%.1f, %.1f) theta=%.1f°  dist_to_target=%.1f cm" %
              (x, y, math.degrees(theta), dist_to_target))
        print("  barriers: %d  closest_obs=%.1f cm" %
              (len(barriers), calculateClosestObstacleDistance(x, y)))
        print("  candidates: %d evaluated, %d clamped" %
              (candidates_evaluated, candidates_clamped))
        print("  chosen: vL=%.2f vR=%.2f  (dps: L=%.0f R=%.0f)" %
              (vLchosen, vRchosen, cm_per_sec_to_dps(vLchosen), cm_per_sec_to_dps(vRchosen)))
        print("  scores: benefit=%.2f  forward=%.2f  move-to-target=%.2f  obs_cost=%.2f  obs_dist=%.1f speed_cost=%.1f" %
              (bestBenefit, best_forward, best_heading, best_obs_cost, best_obs_dist, best_speed_cost))
        if best_obs_dist < SAFEDIST:
            print("  ** AVOIDING OBSTACLE (dist %.1f < safe %.1f) **" %
                  (best_obs_dist, SAFEDIST))

    return vLchosen, vRchosen, debug


def update_pose(x, y, theta, dL_cm, dR_cm):
    """Apply one dead-reckoning step given wheel displacements in cm.
    Returns (x_new, y_new, theta_new)."""
    d_forward = (dL_cm + dR_cm) / 2.0
    d_theta   = (dR_cm - dL_cm) / WHEELBASE_CM
    x_new     = x + d_forward * math.cos(theta + d_theta / 2.0)
    y_new     = y + d_forward * math.sin(theta + d_theta / 2.0)
    theta_new = theta + d_theta
    theta_new = (theta_new + math.pi) % (2 * math.pi) - math.pi
    return x_new, y_new, theta_new

def turnAntiClockwise(particles, angle: float):
    """
    Turn the robot on the spot around its center (between the wheels).
    Positive angle = anticlockwise turn (standard math convention: 0°=right, 90°=up).
    For differential drive turning on the spot:
    - Each wheel travels in an arc with radius = WHEELBASE_WIDTH / 2
    - Arc length = angle_radians * radius
    """
    # Convert angle to radians
    angle_rad = angle * PI / 180.0

    # Calculate arc length each wheel must travel (radius = half wheelbase)
    arc_length = angle_rad * (WHEELBASE_WIDTH + ANGLE_ERROR) / 2.0

    # Convert arc length to encoder degrees
    offset = (arc_length / WHEEL_CIRCUMFERENCE) * 360.0

    try:    # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
        # Reset both encoders to 0 for clean starting positions
        BP.offset_motor_encoder(LEFT_PORT, BP.get_motor_encoder(LEFT_PORT))
        BP.offset_motor_encoder(RIGHT_PORT, BP.get_motor_encoder(RIGHT_PORT))

        # Set speed limits for turning
        BP.set_motor_limits(LEFT_PORT, 50, TURNING_SPEED)
        BP.set_motor_limits(RIGHT_PORT, 50, TURNING_SPEED)

        # Left wheel backward, right wheel forward (for anticlockwise turn)
        left_target = -offset
        right_target = offset

        print("Turning %f degrees anticlockwise\nLeft target: %f, Right target: %f" %
              (angle, left_target, right_target))

        # Set both motor positions (opposite directions for turning on the spot)
        BP.set_motor_position(LEFT_PORT, left_target)
        BP.set_motor_position(RIGHT_PORT, right_target)

        wait_for_motor_position(left_target, right_target)

        # Update particles
        particles.turn(angle)

        print("Turn completed\n")

    except IOError as error:
        print("IOError in turnAntiClockwise: %s" % error)


def wait_for_motor_position(left_target, right_target):
    """
    Wait for motors to reach their target positions.
    Checks actual encoder positions instead of just waiting a fixed time.
    Returns True if targets reached, False if timeout occurred.
    """
    start_time = time.time()
    last_print_time = start_time

    while time.time() - start_time < TIMEOUT:
        try:
            left_current = BP.get_motor_encoder(LEFT_PORT)
            right_current = BP.get_motor_encoder(RIGHT_PORT)

            left_error = abs(left_target - left_current)
            right_error = abs(right_target - right_current)

            # Print progress every 0.5 seconds
            if time.time() - last_print_time > 0.5:
                print("Waiting... L=%d (target=%.1f, error=%.1f), R=%d (target=%.1f, error=%.1f)" %
                      (left_current, left_target, left_error, right_current, right_target, right_error))
                last_print_time = time.time()

            # Check if both motors are within tolerance
            if left_error < POSITION_TOLERANCE and right_error < POSITION_TOLERANCE:
                print("Position reached: L=%d (target=%.1f, error=%.1f), R=%d (target=%.1f, error=%.1f)" %
                      (left_current, left_target, left_error, right_current, right_target, right_error))
                return True

            # Small sleep to avoid hammering the I2C bus
            time.sleep(0.05)

        except IOError as error:
            print("IOError in wait_for_motor_position: %s" % error)
            time.sleep(0.1)

    # Timeout occurred
    try:
        left_current = BP.get_motor_encoder(LEFT_PORT)
        right_current = BP.get_motor_encoder(RIGHT_PORT)
        print("WARNING: Timeout after %d seconds! Current: L=%d (target=%.1f), R=%d (target=%.1f)" %
              (TIMEOUT, left_current, left_target, right_current, right_target))
    except IOError as error:
        print("IOError getting final positions: %s" % error)

    return False

def main():
    global barriers

    # Starting pose (cm, radians)
    x = 0.0
    y = 0.0
    theta = math.pi / 2  # facing +y (toward target)

    # Initial velocities (cm/s)
    vL = 0.0
    vR = 0.0

    # Reset encoders to zero
    BP.offset_motor_encoder(LEFT_PORT, BP.get_motor_encoder(LEFT_PORT))
    BP.offset_motor_encoder(RIGHT_PORT, BP.get_motor_encoder(RIGHT_PORT))

    try:
        while True:
            loop_start = time.time()

            # --- Check goal ---
            dist_to_target = math.sqrt((x - target[0])**2 + (y - target[1])**2)
            if dist_to_target < GOAL_TOLERANCE:
                BP.set_motor_dps(LEFT_PORT, 0)
                BP.set_motor_dps(RIGHT_PORT, 0)
                print("Target reached!")
                break

            # --- Camera detection ---
            (img, canCentroids) = captureCanCentroids(picam2)
            img = drawGridOnImage(img)

            print("--- CAM ---")
            print("  detected: %d cans" % len(canCentroids))

            for (*_, lowest_point) in canCentroids:
                # Transform pixel coords to camera-frame ground plane (cm)
                (cam_x, cam_y) = HtransformUVtoXY(HInv, lowest_point[0], lowest_point[1])
                if cam_x > CAM_DIST: # centroid is too far away, consider if closer
                    continue

                # Transform camera-frame to world-frame using robot pose + heading
                world_x = x + cam_x * math.cos(theta) + cam_y * math.sin(theta)
                world_y = y + cam_x * math.sin(theta) - cam_y * math.cos(theta)
                add_barrier(world_x, world_y)

                # Draw detection on image
                display_x, display_y = int(lowest_point[0]), int(lowest_point[1])
                img = cv2.circle(img, (display_x, display_y), 5, WHITE, 3)
                pstring = "(%.0f, %.0f)" % (world_x, world_y)
                img = cv2.putText(img, pstring, (display_x + 8, display_y), FONT, 0.5, WHITE, 1, cv2.LINE_AA)

                print("  can: px=(%.0f,%.0f) cam=(%.1f,%.1f) world=(%.1f,%.1f)" %
                      (lowest_point[0], lowest_point[1], cam_x, cam_y, world_x, world_y))

            displayImg(img)

            # Cap barrier list to prevent stale detections accumulating
            if len(barriers) > MAX_BARRIERS:
                barriers = barriers[-MAX_BARRIERS:]

            # --- DWA Planning ---
            vL, vR, _ = dwa_choose_velocities(x, y, theta, vL, vR, verbose=True)

            # --- Motor execution ---
            BP.set_motor_dps(LEFT_PORT, cm_per_sec_to_dps(vL))
            BP.set_motor_dps(RIGHT_PORT, cm_per_sec_to_dps(vR))

            # Read encoders before sleep
            enc_left_before = BP.get_motor_encoder(LEFT_PORT)
            enc_right_before = BP.get_motor_encoder(RIGHT_PORT)

            elapsed = time.time() - loop_start
            time.sleep(max(0, dt - elapsed))

            # Read encoders after sleep
            enc_left_after = BP.get_motor_encoder(LEFT_PORT)
            enc_right_after = BP.get_motor_encoder(RIGHT_PORT)

            # Compute actual wheel displacements (cm)
            dL = encoder_deg_to_cm(enc_left_after - enc_left_before)
            dR = encoder_deg_to_cm(enc_right_after - enc_right_before)

            # Update pose via dead reckoning
            x, y, theta = update_pose(x, y, theta, dL, dR)

            loop_time = time.time() - loop_start
            print("--- ODOM ---")
            print("  encoders: dL=%.1f° dR=%.1f°  -> dL=%.2f cm dR=%.2f cm" %
                  (enc_left_after - enc_left_before, enc_right_after - enc_right_before, dL, dR))
            print("  dead_reck: dL=%.2f cm  dR=%.2f cm" % (dL, dR))
            print("  new_pose: (%.1f, %.1f) theta=%.1f°  dist=%.1f cm" %
                  (x, y, math.degrees(theta), dist_to_target))
            print("  loop_time: %.0f ms" % (loop_time * 1000))

            # --- Bump sensor polling (TODO: enable when sensors are connected) ---
            try:
                left_touch = BP.get_sensor(LEFT_TOUCH_PORT)
                right_touch = BP.get_sensor(RIGHT_TOUCH_PORT)
                if left_touch or right_touch:
                    # Stop immediately
                    BP.set_motor_dps(LEFT_PORT, 0)
                    BP.set_motor_dps(RIGHT_PORT, 0)
                    time.sleep(0.2)
                    # Reverse ~5cm (50mm)
                    from particleDataStructures import Particles, Canvas
                    # motion.forward(particles, -50)
                    # Turn 45° away from the hit side
                    if left_touch:
                        turnAntiClockwise(None, -45)  # turn clockwise
                    else:
                        turnAntiClockwise(None, 45)   # turn anticlockwise
                    # Re-read pose from encoders after recovery manoeuvre
            except (BP_SENSOR_ERROR, IOError):
                pass

    finally:
        BP.reset_all()
        picam2.stop()
        print("Motors reset. Camera stopped.")


if __name__ == "__main__":
    main()
