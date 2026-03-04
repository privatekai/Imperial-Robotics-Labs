import brickpi3
import math
import cv2
import numpy as np
from motion import forward, turnAntiClockwise
from picamera2 import Picamera2
from picameracans import captureCanCentroids, displayImg, WHITE, FONT
from picamerahomographygrid import drawGridOnImage, HtransformXYtoUV, HtransformUVtoXY, HInv

from lin_alg import projection, magnitude

BARRIER_RADIUS = 3.3
RADIUS = 2
ROBOT_RADIUS = 10

FORWARD_THETA = 0

SEMICIRCLE_FIDELITY = 11 # number of evaluated points on the evaluated the circle
SEMICIRCLE_RADIUS = 5 # in cm
SEMICIRCLE_RANGE = 180 # range of angles in the semicircle
SEMICIRCLE_STEP = SEMICIRCLE_RANGE / (SEMICIRCLE_FIDELITY - 1)

CAN_X_UNCERTAINTY = 3
CAN_Y_UNCERTAINTY = 15

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
        
    least_angle = theta - SEMICIRCLE_RANGE / 2
    
    semicircle_positions = []
    
    for i in range(SEMICIRCLE_FIDELITY):
        cur_angle = least_angle + SEMICIRCLE_STEP * i
        cur_rads = math.radians(cur_angle)
        
        new_x = x + SEMICIRCLE_RADIUS * math.cos(cur_rads)
        new_y = y + SEMICIRCLE_RADIUS * math.sin(cur_rads)
        
        semicircle_positions.append((new_x, new_y, cur_angle))
    
    return semicircle_positions

def scorePosition(new_x, new_y, barriers, x = 0, y = 0):

    # math math math...
    closest_barrier = calculateClosestObstacleDistance(x, y, barriers)
    c_x, c_y = closest_barrier # assuming x = 0, y = 0 for the can coordinates
    new_x -= x
    new_y -= y
    proj_x, proj_y = projection(new_x, new_y, c_x, c_y)
    distance = magnitude(proj_x - c_x, proj_y - c_y)
    
    # Calculate score
    score = new_x * math.cos(math.radians(FORWARD_THETA)) + new_y * math.sin(math.radians(FORWARD_THETA))
    if distance < BARRIER_RADIUS + ROBOT_RADIUS: 
        score = float("-inf")
    elif CAN_X_UNCERTAINTY - distance > 0: # Pick better x uncertainty here
        score -= CAN_X_UNCERTAINTY - distance # Cost of hitting can

    print("x: " + new_x + " y: " + new_y)
    print("score: ", score)

    return score


# Function to calculate the closest obstacle at a position (x, y)
# Used during planning
def calculateClosestObstacleDistance(x, y, barriers):
    # TODO: Remove if statement about whether we know about the barrier or not.
    closest_dist = float("inf")
    closest_barrier = None
    # Calculate distance to closest obstacle
    for barrier in barriers:
        # Is this a barrier we know about? barrier[2] flag is set when sonar observes it
        dx = barrier[0] - x
        dy = barrier[1] - y
        d = math.sqrt(dx**2 + dy**2)
        # Distance between closest touching point of circular robot and circular barrier
        dist = d - BARRIER_RADIUS - ROBOT_RADIUS
        if (dist < closest_dist):
            closest_dist = dist
            closest_barrier = barrier
    return closest_barrier

semicircle_positions = semicircle()

# Main loop
while(1):
    # Check if any new barriers are visible from current pose -> i.e. run our camera object detection code here
    (img, canCentroids) = captureCanCentroids(picam2)
    drawGridOnImage(img)

    # Note: Probably still need another fail safe to ensure that we don't add extra barriers
    # But should be okay for now
    barriers = [HtransformUVtoXY(HInv, lowest_point[1], -lowest_point[0]) for (*_ , lowest_point) in canCentroids]

    best_score_index = None
    best_score = 0
    for i in range(len(semicircle_positions)):
        (x,y,angle) = semicircle_positions[i]
        score = scorePosition(x, y, barriers)
        if score > best_score:
            best_score_index = i
            best_score = score

    _, _, angle = semicircle_positions[best_score_index]

    # TODO: Turn, move forward, Turn back

    turnAntiClockwise(angle)
    forward(SEMICIRCLE_RADIUS * 10)
    turnAntiClockwise(-angle)
    
    # for (x, y, w, h, area, lowest_point) in canCentroids:

    #     # This is relevant to the camera coords - make it in perspective to the robot position
    #     (lowest_x, lowest_y) =  HtransformUVtoXY(HInv, lowest_point[0], lowest_point[1])
    #     (lowest_x, lowest_y) =  (lowest_x + x, lowest_y + y)

    #     # Update to barriers
    #     i = 0
    #     while i < len(barriers):
    #         (barrier_x, barrier_y) = barriers[i]
    #         # Check if barrier already exists in barriers
    #         if abs(barrier_x - lowest_x) < BARRIER_RADIUS + B_UNCERTAINTY and abs(barrier_y - lowest_y) < RADIUS + B_UNCERTAINTY:
    #             barriers.remove(i)
    #             barriers.append((lowest_x, lowest_y))
    #             break
    #         i += 1
        
    #     # Add to barriers if not there before
    #     if i == len(barriers):
    #         barriers.append((lowest_x, lowest_y))

    #     # Coords to draw blob on image output
    #     display_x, display_y = int(lowest_point[0]), int(lowest_point[1])

    #     # Draw a little circle to show each detected blob
    #     img = cv2.circle(img, (display_x, display_y), 5, WHITE, 3)

    #     # Also print its coordinates on the image
    #     pstring = "(" + str(lowest_x) + "," + str(lowest_y) + ")"
    #     img = cv2.putText(img, pstring, (display_x + 8, display_y), FONT, 0.5, WHITE, 1, cv2.LINE_AA)
        
    #     print(lowest_point, "-->", barriers[-1])

    # displayImg(img) # Prints to web interface

    # Planning
    # We want to find the best benefit where we have a positive component for closeness to target,
    # and a negative component for closeness to obstacles, for each of a choice of possible actions

picam2.stop()
    # TODO: move forward using vL and vR
    # TODO: (optional) Check if we are touching something, and readjust barriers list if so, as well as correct current position.
    # TODO: Check if we are at target, as weloop this function until at target