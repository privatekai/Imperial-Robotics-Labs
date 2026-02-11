import time
import numpy as np
from questions3 import BP, LEFT_MOTOR_PORT, MOVEMENT_SPEED, RIGHT_MOTOR_PORT, forward, turnAntiClockwise
from visualisation import NUM_PARTICLES, ROBOT_START_POS, initial_drawing, robot_position

def navigate_to_waypoint(waypoint, particles, weights):
    # mean of the particles X, Y and theta
    # find angle and distance to get to waypoint
    # move robot (this will update the particles).
    robot_x, robot_y, robot_facing = robot_position(particles, weights)
    w_x, w_y = waypoint

    print("robot_x: ", robot_x)
    print("robot_y: ", robot_y)
    print("robot_facing: ", robot_facing)

    print("w_x: ", w_x)
    print("w_y: ", w_y)

    distance = np.sqrt((w_x - robot_x)**2 + (w_y - robot_y)**2)
    print("distance: ", distance)

    print("facing_target_rad:", np.arctan((w_y - robot_y)/(w_x - robot_x)))
    facing_target = np.rad2deg(np.arctan((w_y - robot_y) / (w_x - robot_x)))
    if (w_x - robot_x) < 0:
        facing_target += 180
    print("facing_target: ", facing_target)
    print("turn: ", facing_target-robot_facing)

    particles = turnAntiClockwise(particles, facing_target-robot_facing)
    particles = forward(particles, distance)

    return particles

try:
    try:
        BP.offset_motor_encoder(LEFT_MOTOR_PORT, BP.get_motor_encoder(LEFT_MOTOR_PORT)) # reset encoder A
        BP.offset_motor_encoder(RIGHT_MOTOR_PORT, BP.get_motor_encoder(RIGHT_MOTOR_PORT)) # reset encoder D
    except IOError as error:
        print(error)
    
    # Initial motor limits (will be updated in forward() and turnAntiClockwise())
    BP.set_motor_limits(LEFT_MOTOR_PORT, 50, MOVEMENT_SPEED)
    BP.set_motor_limits(RIGHT_MOTOR_PORT, 50, MOVEMENT_SPEED)

    particles = np.array([ROBOT_START_POS] * NUM_PARTICLES)
    weights = np.array([1/NUM_PARTICLES] * NUM_PARTICLES)
    
    initial_drawing(particles)

    time.sleep(1)

    particles = navigate_to_waypoint((80, 40), particles, weights)
    BP.reset_all()
    try:
        BP.offset_motor_encoder(LEFT_MOTOR_PORT, BP.get_motor_encoder(LEFT_MOTOR_PORT)) # reset encoder A
        BP.offset_motor_encoder(RIGHT_MOTOR_PORT, BP.get_motor_encoder(RIGHT_MOTOR_PORT)) # reset encoder D
    except IOError as error:
        print(error)

    # Initial motor limits (will be updated in forward() and turnAntiClockwise())
    BP.set_motor_limits(LEFT_MOTOR_PORT, 50, MOVEMENT_SPEED)
    BP.set_motor_limits(RIGHT_MOTOR_PORT, 50, MOVEMENT_SPEED)

    particles = navigate_to_waypoint((0, 0), particles, weights)

finally: # at the end of everything, even with exception.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.