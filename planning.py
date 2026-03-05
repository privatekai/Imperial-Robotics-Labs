import brickpi3
import math
import cv2
import numpy as np
from motion import forward, reset_motors, turnAntiClockwise
from picamera2 import Picamera2
from picameracans import captureCanCentroids, displayImg, WHITE, FONT
from picamerahomographygrid import drawGridOnImage, HtransformXYtoUV, HtransformUVtoXY, HInv

from lin_alg import projection, magnitude

BARRIER_RADIUS = 3.3
RADIUS = 2
ROBOT_RADIUS = 10
MOVEMENT_DIST = 20

FORWARD_THETA = 0

SEMICIRCLE_FIDELITY = 11 # number of evaluated points on the evaluated the circle
SEMICIRCLE_RADIUS = 35 # in cm
SEMICIRCLE_RANGE = 180 # range of angles in the semicircle
SEMICIRCLE_STEP = SEMICIRCLE_RANGE / (SEMICIRCLE_FIDELITY - 1)

CAN_H_UNCERTAINTY = 3
CAN_V_UNCERTAINTY = 15

# Needed constants
# TODO: Set these constants.
# BARRIERTHRESHOLD - threshold for can area to be considered a barrier and merged with other detected cans
# W - robot width (0.5 * robot radius)
# ROBOTRADIUS - robot radius
# BARRIERRADIUS - width of barrier
# k = 160 - pixels per metre for graphics
# set the width and height of the screen (pixels)
# WIDTH = 1500
# HEIGHT = 1000
# Screen centre will correspond to (x, y) = (0, 0)
# u0 = WIDTH / 2
# v0 = HEIGHT / 2

# MAXVELOCITY - max safe velocity of robot
# MAXACCELERATION - max safe acceleration of robot
# SAFEDIST - safe distance away from barrier

# Start camera
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(preview_config)
picam2.start()

# Timestep delta to run control at
dt = 0.1

# Starting pose of robot
x = 0.0
y = 0.0
theta = 0.0

# Barrier (obstacle) locations
# A barrier should be a tuple (x, y), where (x, y) is the centre of the barrier. Barriers are not added to this list until seen with the camera.
# barriers = []

# Set an initial target location which is beyond the obstacles
# TODO: Set this variable to something accurate to the location of the target in the real life course.
target = (0, 400)

def semicircle(x = 0, y = 0, theta = FORWARD_THETA):
    if theta != FORWARD_THETA:
        print("semicircle, WARNING: ROBOT SHOULD BE FACING FORWARDS!")
        
    least_angle = theta + SEMICIRCLE_RANGE / 2
    
    semicircle_positions = []
    
    for i in range(SEMICIRCLE_FIDELITY):
        cur_angle = least_angle - SEMICIRCLE_STEP * i
        cur_rads = math.radians(cur_angle)

        print(f"CUR ANGLE: {cur_angle}")
        
        new_x = x + SEMICIRCLE_RADIUS * math.sin(cur_rads)
        new_y = y + SEMICIRCLE_RADIUS * math.cos(cur_rads)
        
        semicircle_positions.append((new_x, new_y, cur_angle))
    
    return semicircle_positions

def scorePosition(pred_h, pred_v, barriers):
    score = pred_v

    closest_barriers = calculate3ClosestObstacleDistance(pred_v, pred_h, barriers)
    if not closest_barriers:
        return score
    
    for closest_barrier in closest_barriers:
        b_v, b_h, b_w = closest_barrier # assuming x = 0, y = 0 for the can coordinates

        h1 = b_h - b_w/2 - ROBOT_RADIUS
        h2 = b_h + b_w/2 + ROBOT_RADIUS
        v = b_v - BARRIER_RADIUS - ROBOT_RADIUS

        if pred_h > h1 and pred_h < h2 and pred_v > v:
            score = float("-inf")

    return score


# Function to calculate the closest obstacle at a position (x, y)
# Used during planning
# RETURNS IN BARRIER FORMAT!?
def calculateClosestObstacleDistance(vertical, horizontal, barriers):
    # TODO: Remove if statement about whether we know about the barrier or not.
    closest_dist = float("inf")
    closest_barrier = None
    # Calculate distance to closest obstacle
    for barrier in barriers:
        # Is this a barrier we know about? barrier[2] flag is set when sonar observes it
        dv = barrier[0] - vertical
        dh = barrier[1] - horizontal
        d = math.sqrt(dv**2 + dh**2)
        # Distance between closest touching point of circular robot and circular barrier
        dist = d - BARRIER_RADIUS - ROBOT_RADIUS
        if (dist < closest_dist):
            closest_dist = dist
            closest_barrier = barrier
    return closest_barrier

def calculate3ClosestObstacleDistance(vertical, horizontal, barriers):
    def dist(x, y):
        dv = barrier[0] - vertical
        dh = barrier[1] - horizontal
        d = math.sqrt(dv**2 + dh**2)
        # Distance between closest touching point of circular robot and circular barrier
        dist = d - BARRIER_RADIUS - ROBOT_RADIUS
        return dist


    barriers.sort(key=lambda b: dist(b[0],b[1]))
    to_take = min(len(barriers), 3)
    closest_barriers3 = barriers[:to_take]

    return closest_barriers3

semicircle_positions = semicircle()

if __name__ == "__main__":
    try:
        # Main loop
        debug_i = 0
        while(1):
            # Check if any new barriers are visible from current pose -> i.e. run our camera object detection code here
            (img, canCentroids) = captureCanCentroids(picam2)
            drawGridOnImage(img)

            # Note: Probably still need another fail safe to ensure that we don't add extra barriers
            # But should be okay for now
            # barriers = [(HtransformUVtoXY(HInv, lowest_point[0], lowest_point[1])) for (*_ , lowest_point) in canCentroids]
            barriers = []
            for (_, _, w, _, _, lowest_point) in canCentroids:
                (v, h) = HtransformUVtoXY(HInv, lowest_point[0], lowest_point[1])
                barriers.append((v, h, w))
                
            # barrier: (vertical, horizontal)

            f_type = "w" if debug_i == 0 else "a"
            with open("barriers_out.txt", f_type) as f:
                f.write("--- BARRIERS --- (note: barrier format)\n")
                f.write("loop number " + str(debug_i) + "\n")
                for barrier in barriers:
                    f.write(str(barrier) + "\n")
                f.close()

            best_angle = 0
            best_score = float("-inf")
            for i in range(len(semicircle_positions)):
                (horizontal, vertical, angle) = semicircle_positions[i]
                score = scorePosition(horizontal, vertical, barriers)

                print(f"horizontal: {horizontal}, vertical: {vertical}, theta: {angle}")
                print("score: ", score)
                if score > best_score:
                    best_angle = angle
                    best_score = score

            print(f"SELECTED MOV - theta: {best_angle}")

            turnAntiClockwise(best_angle)
            forward(MOVEMENT_DIST * 10)
            turnAntiClockwise(-best_angle)

            debug_i += 1

    finally:
        reset_motors()
        picam2.stop()
        print("Motors reset. Camera stopped.")

    # TODO: move forward using vL and vR
    # TODO: (optional) Check if we are touching something, and readjust barriers list if so, as well as correct current position.
    # TODO: Check if we are at target, as weloop this function until at target