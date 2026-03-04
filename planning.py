import brickpi3
import time
import math
import cv2
import numpy as np
from picamera2 import Picamera2
from picameracans import captureCanCentroids, displayImg, WHITE, FONT
from picamerahomographygrid import drawGridOnImage, HtransformXYtoUV, HtransformUVtoXY, HInv
# from motion import forward, turnAntiClockwise

B_UNCERTAINTY = 2
RADIUS = 2

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
barriers = []

# Set an initial target location which is beyond the obstacles
# TODO: Set this variable to something accurate to the location of the target in the real life course.
target = (0, 400)

# I think can be replaced with our own position / odometry functions
# Function to predict new robot position based on current pose and velocity controls
# Uses time deltat in future
# Returns xnew, ynew, thetanew
# Also returns path. This is just used for graphics, and returns some complicated stuff
# used to draw the possible paths during planning. Don't worry about the details of that.
def predictPosition(vL, vR, x, y, theta, deltat):
    # TODO: Change this function to get rid of drawing/pygame references and variables we don't need!
    # Simple special cases
    # Straight line motion
    if (vL == vR): 
        xnew = x + vL * deltat * math.cos(theta)
        ynew = y + vL * deltat * math.sin(theta)
        thetanew = theta
        path = (0, vL * deltat)   # 0 indicates pure translation
    # Pure rotation motion
    elif (vL == -vR):
        xnew = x
        ynew = y
        thetanew = theta + ((vR - vL) * deltat / W)
        path = (1, 0) # 1 indicates pure rotation
    else:
        # Rotation and arc angle of general circular motion
        # Using equations given in Lecture 2
        R = W / 2.0 * (vR + vL) / (vR - vL)
        deltatheta = (vR - vL) * deltat / W
        xnew = x + R * (math.sin(deltatheta + theta) - math.sin(theta))
        ynew = y - R * (math.cos(deltatheta + theta) - math.cos(theta))
        thetanew = theta + deltatheta

        # To calculate parameters for arc drawing (complicated Pygame stuff, don't worry)
        # We need centre of circle
        (cx, cy) = (x - R * math.sin(theta), y + R * math.cos (theta))
        # Turn this into Rect
        Rabs = abs(R)
        ((tlx, tly), (Rx, Ry)) = ((int(u0 + k * (cx - Rabs)), int(v0 - k * (cy + Rabs))), (int(k * (2 * Rabs)), int(k * (2 * Rabs))))
        if (R > 0):
            start_angle = theta - math.pi/2.0
        else:
            start_angle = theta + math.pi/2.0
        stop_angle = start_angle + deltatheta
        path = (2, ((tlx, tly), (Rx, Ry)), start_angle, stop_angle) # 2 indicates general motion

    return (xnew, ynew, thetanew, path)

# Function to calculate the closest obstacle at a position (x, y)
# Used during planning
def calculateClosestObstacleDistance(x, y):
    # TODO: Remove if statement about whether we know about the barrier or not.
    closestdist = 100000.0  
    # Calculate distance to closest obstacle
    for barrier in barriers:
        # Is this a barrier we know about? barrier[2] flag is set when sonar observes it
        if(barrier[2] == 1):
            dx = barrier[0] - x
            dy = barrier[1] - y
            d = math.sqrt(dx**2 + dy**2)
            # Distance between closest touching point of circular robot and circular barrier
            dist = d - BARRIERRADIUS - ROBOTRADIUS
            if (dist < closestdist):
                    closestdist = dist
    return closestdist


# Main loop
while(1):
    # Check if any new barriers are visible from current pose -> i.e. run our camera object detection code here
    (img, canCentroids) = captureCanCentroids(picam2)
    drawGridOnImage(img)

    for (x, y, w, h, area, lowest_point) in canCentroids:

        # This is relevant to the camera coords - make it in perspective to the robot position
        (lowest_x, lowest_y) =  HtransformUVtoXY(HInv, lowest_point[0], lowest_point[1])
        (lowest_x, lowest_y) =  (lowest_x + x, lowest_y + y)

        # Update to barriers
        i = 0
        while i < len(barriers):
            (barrier_x, barrier_y) = barriers[i]
            # Check if barrier already exists in barriers
            if abs(barrier_x - lowest_x) < RADIUS + B_UNCERTAINTY and abs(barrier_y - lowest_y) < RADIUS + B_UNCERTAINTY:
                barriers.remove(i)
                barriers.append((lowest_x, lowest_y))
                break
            i += 1
        
        # Add to barriers if not there before
        if i == len(barriers):
            barriers.append((lowest_x, lowest_y))

        # Coords to draw blob on image output
        display_x, display_y = int(lowest_point[0]), int(lowest_point[1])

        # Draw a little circle to show each detected blob
        img = cv2.circle(img, (display_x, display_y), 5, WHITE, 3)

        # Also print its coordinates on the image
        pstring = "(" + str(lowest_x) + "," + str(lowest_y) + ")"
        img = cv2.putText(img, pstring, (display_x + 8, display_y), FONT, 0.5, WHITE, 1, cv2.LINE_AA)
        
        print(lowest_point, "-->", barriers[-1])

    displayImg(img) # Prints to web interface

    

    # Planning
    # We want to find the best benefit where we have a positive component for closeness to target,
    # and a negative component for closeness to obstacles, for each of a choice of possible actions

    bestBenefit = -100000
    FORWARDWEIGHT = 12
    OBSTACLEWEIGHT = 16

    # TODO: Change this loop to get rid of references to drawing and pygame variables we don't need, and use our own modified above functions
    # Range of possible motions: each of vL and vR could go up or down a bit
    vLpossiblearray = (vL - MAXACCELERATION * dt, vL, vL + MAXACCELERATION * dt)
    vRpossiblearray = (vR - MAXACCELERATION * dt, vR, vR + MAXACCELERATION * dt)
    print ("New action")
    newpositionstodraw = [] # Also for possible plotting of robot end positions
    for vLpossible in vLpossiblearray:
        for vRpossible in vRpossiblearray:
                # We can only choose an action if it's within velocity limits
                if (vLpossible <= MAXVELOCITY and vRpossible <= MAXVELOCITY and vLpossible >= -MAXVELOCITY and vRpossible >= -MAXVELOCITY):
                    # Predict new position in TAU seconds
                    TAU = 1.5 
                    (xpredict, ypredict, thetapredict, path) = predictPosition(vLpossible, vRpossible, x, y, theta, TAU)
                    newpositionstodraw.append((xpredict, ypredict))
                    # What is the distance to the closest obstacle from this possible position?
                    distanceToObstacle = calculateClosestObstacleDistance(xpredict, ypredict)
                    # Calculate how much close we've moved to target location
                    previousTargetDistance = math.sqrt((x - target[0])**2 + (y - target[1])**2)
                    newTargetDistance = math.sqrt((xpredict - target[0])**2 + (ypredict - target[1])**2)
                    distanceForward = previousTargetDistance - newTargetDistance
                    # Alternative: how far have I moved forwards?
                    # distanceForward = xpredict - x
                    # Positive benefit
                    distanceBenefit = FORWARDWEIGHT * distanceForward
                    # Negative cost: once we are less than SAFEDIST from collision, linearly increasing cost
                    if (distanceToObstacle < SAFEDIST):
                        obstacleCost = OBSTACLEWEIGHT * (SAFEDIST - distanceToObstacle)
                    else:
                        obstacleCost = 0.0
                    # Total benefit function to optimise
                    benefit = distanceBenefit - obstacleCost
                    if (benefit > bestBenefit):
                        vLchosen = vLpossible
                        vRchosen = vRpossible
                        bestBenefit = benefit
    vL = vLchosen
    vR = vRchosen

picam2.stop()
    # TODO: move forward using vL and vR
    # TODO: (optional) Check if we are touching something, and readjust barriers list if so, as well as correct current position.
    # TODO: Check if we are at target, as weloop this function until at target