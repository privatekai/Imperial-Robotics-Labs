import math, time

import cv2
from picamera2 import Picamera2
from picameracans import captureCanCentroids, displayImg, FONT
from picamerahomographygrid import drawGridOnImage, HtransformUVtoXY, HInv

from constants import (
    BP, LEFT_MOTOR_PORT as LEFT_PORT, RIGHT_MOTOR_PORT as RIGHT_PORT,
    LEFT_TOUCH_PORT, RIGHT_TOUCH_PORT,
    WHEEL_CIRCUMFERENCE_CM, WHEELBASE_CM,
    ROBOTRADIUS, BARRIERRADIUS, SAFEDIST,
    MAXVELOCITY, MAXACCELERATION,
    GOAL_TOLERANCE, MAX_BARRIERS,
    FORWARDWEIGHT, OBSTACLEWEIGHT, SPEEDWEIGHT, TAU,
    DEDUP_RADIUS, WHITE,
)

# Start camera
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(preview_config)
picam2.start()

# Timestep delta to run control at
dt = 0.2

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
        if math.sqrt((bx - world_x)**2 + (by - world_y)**2) < DEDUP_RADIUS:
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
        dist = d - BARRIERRADIUS - ROBOTRADIUS
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
    vLpossiblearray = [vL + s for s in steps]
    vRpossiblearray = [vR + s for s in steps]

    candidates_evaluated = 0
    candidates_clamped = 0
    best_forward = 0.0
    best_obs_cost = 0.0
    best_obs_dist = float('inf')

    for vLpossible in vLpossiblearray:
        for vRpossible in vRpossiblearray:
            if abs(vLpossible) <= MAXVELOCITY and abs(vRpossible) <= MAXVELOCITY:
                candidates_evaluated += 1
                (xpredict, ypredict, _) = predictPosition(
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

                benefit = distanceBenefit - obstacleCost - speedCost
                if benefit > bestBenefit:
                    vLchosen = vLpossible
                    vRchosen = vRpossible
                    bestBenefit = benefit
                    best_forward = distanceBenefit
                    best_obs_cost = obstacleCost
                    best_obs_dist = distanceToObstacle
            else:
                candidates_clamped += 1

    debug = {
        'candidates_evaluated': candidates_evaluated,
        'candidates_clamped': candidates_clamped,
        'bestBenefit': bestBenefit,
        'best_forward': best_forward,
        'best_obs_cost': best_obs_cost,
        'best_obs_dist': best_obs_dist,
    }

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
        print("  scores: benefit=%.2f  forward=%.2f  obs_cost=%.2f  obs_dist=%.1f" %
              (bestBenefit, best_forward, best_obs_cost, best_obs_dist))
        if best_obs_dist < SAFEDIST:
            print("  ** AVOIDING OBSTACLE (dist %.1f < safe %.1f) **" %
                  (best_obs_dist, SAFEDIST))

    return vLchosen, vRchosen, debug


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

                # Transform camera-frame to world-frame using robot pose + heading
                # cam_x = forward (along robot facing), cam_y = lateral
                world_x = x + cam_x * math.cos(theta) - cam_y * math.sin(theta)
                world_y = y + cam_x * math.sin(theta) + cam_y * math.cos(theta)
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
            d_forward = (dL + dR) / 2.0
            d_theta = (dR - dL) / WHEELBASE_CM

            x += d_forward * math.cos(theta + d_theta / 2.0)
            y += d_forward * math.sin(theta + d_theta / 2.0)
            theta += d_theta
            theta = (theta + math.pi) % (2 * math.pi) - math.pi

            loop_time = time.time() - loop_start
            print("--- ODOM ---")
            print("  encoders: dL=%.1f° dR=%.1f°  -> dL=%.2f cm dR=%.2f cm" %
                  (enc_left_after - enc_left_before, enc_right_after - enc_right_before, dL, dR))
            print("  dead_reck: fwd=%.2f cm  dtheta=%.2f°" %
                  (d_forward, math.degrees(d_theta)))
            print("  new_pose: (%.1f, %.1f) theta=%.1f°  dist=%.1f cm" %
                  (x, y, math.degrees(theta), dist_to_target))
            print("  loop_time: %.0f ms" % (loop_time * 1000))

            # --- Bump sensor polling (TODO: enable when sensors are connected) ---
            # try:
            #     left_touch = BP.get_sensor(LEFT_TOUCH_PORT)
            #     right_touch = BP.get_sensor(RIGHT_TOUCH_PORT)
            #     if left_touch or right_touch:
            #         # Stop immediately
            #         BP.set_motor_dps(LEFT_PORT, 0)
            #         BP.set_motor_dps(RIGHT_PORT, 0)
            #         time.sleep(0.2)
            #         # Reverse ~5cm (50mm)
            #         from particleDataStructures import Particles, Canvas
            #         # motion.forward(particles, -50)
            #         # Turn 45° away from the hit side
            #         if left_touch:
            #             motion.turnAntiClockwise(None, -45)  # turn clockwise
            #         else:
            #             motion.turnAntiClockwise(None, 45)   # turn anticlockwise
            #         # Re-read pose from encoders after recovery manoeuvre
            # except (brickpi3.SensorError, IOError):
            #     pass

    finally:
        BP.reset_all()
        picam2.stop()
        print("Motors reset. Camera stopped.")


if __name__ == "__main__":
    main()
