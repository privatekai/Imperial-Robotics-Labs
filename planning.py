import os, math, time, random

# Needed constants
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

# Timestep delta to run control at
dt = 0.1

# Starting pose of robot
x = 0.0
y = 0.0
theta = 0.0

# Barrier (obstacle) locations
barriers = []
# add walls of arena ?

# Set an initial target location which is beyond the obstacles
target = (0, 400)

# I think can be replaced with our own position / odometry functions
# Function to predict new robot position based on current pose and velocity controls
# Uses time deltat in future
# Returns xnew, ynew, thetanew
# Also returns path. This is just used for graphics, and returns some complicated stuff
# used to draw the possible paths during planning. Don't worry about the details of that.
def predictPosition(vL, vR, x, y, theta, deltat):
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
    # For display of trail
    # locationhistory.append((x, y))
    
    # Check if any new barriers are visible from current pose -> i.e. run our camera object detection code here
    # observeBarriers(x, y, theta)

    # Planning
    # We want to find the best benefit where we have a positive component for closeness to target,
    # and a negative component for closeness to obstacles, for each of a choice of possible actions
    bestBenefit = -100000
    FORWARDWEIGHT = 12
    OBSTACLEWEIGHT = 16

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

    # then move forward using vL and vR
    # then loop this function until at target