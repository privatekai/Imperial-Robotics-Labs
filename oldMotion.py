from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

# UNITS ARE MILLIMETRES

MOVEMENT_SPEED = 180  # Speed for moving forward (Degrees Per Second)
TURNING_SPEED = 150   # Speed for turning (Degrees Per Second)

PI = 3.14159627

WHEEL_DIAMETER = 69.36
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI
WHEELBASE_WIDTH = 134.341

MINI_WAIT_TIME = 3  # Time to wait after each movement (Seconds)

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

LEFT_MOTOR_PORT = BP.PORT_A
RIGHT_MOTOR_PORT = BP.PORT_D

PAUSE = True  # Whether to pause and wait for each movement to complete

def forward(distance: float):
    offset = (360 * distance) / WHEEL_CIRCUMFERENCE
    try:
        BP.set_motor_position(LEFT_MOTOR_PORT, BP.get_motor_encoder(LEFT_MOTOR_PORT) + offset)
        BP.set_motor_position(RIGHT_MOTOR_PORT, BP.get_motor_encoder(RIGHT_MOTOR_PORT) + offset)
    except IOError as error:
        print(error)
    
    try:
        print("Moving forward %f mm\nA offset %f -> %f\nD offset %f -> %f" % (distance, BP.get_motor_encoder(LEFT_MOTOR_PORT), BP.get_motor_encoder(LEFT_MOTOR_PORT) + offset, BP.get_motor_encoder(RIGHT_MOTOR_PORT), BP.get_motor_encoder(RIGHT_MOTOR_PORT) + offset))
    except IOError as error:
        print(error)

    if PAUSE: time.sleep(abs(distance / (MOVEMENT_SPEED * WHEEL_CIRCUMFERENCE / 360)) + MINI_WAIT_TIME)  # wait for movement to complete


def turnClockwise(angle: float):
    offset = (WHEELBASE_WIDTH * PI * angle / 360) / WHEEL_CIRCUMFERENCE * 360
    try:
        BP.set_motor_position(LEFT_MOTOR_PORT, BP.get_motor_encoder(LEFT_MOTOR_PORT) + offset)
        BP.set_motor_position(RIGHT_MOTOR_PORT, BP.get_motor_encoder(RIGHT_MOTOR_PORT) - offset)

    except IOError as error:
        print(error)
    
    try:
        print("Turning clockwise %f degrees\nA offset %f -> %f\nD offset %f -> %f" % (angle, BP.get_motor_encoder(LEFT_MOTOR_PORT), BP.get_motor_encoder(LEFT_MOTOR_PORT) + offset, BP.get_motor_encoder(RIGHT_MOTOR_PORT), BP.get_motor_encoder(RIGHT_MOTOR_PORT) - offset))
    except IOError as error:
        print(error)

    if PAUSE: time.sleep(abs((WHEELBASE_WIDTH * PI * angle / 360) / (TURNING_SPEED * WHEEL_CIRCUMFERENCE / 360)) + MINI_WAIT_TIME)  # wait for movement to complete

try:
    try:
        BP.offset_motor_encoder(LEFT_MOTOR_PORT, BP.get_motor_encoder(LEFT_MOTOR_PORT)) # reset encoder A
        BP.offset_motor_encoder(RIGHT_MOTOR_PORT, BP.get_motor_encoder(RIGHT_MOTOR_PORT)) # reset encoder D
    except IOError as error:
        print(error)
    
    BP.set_motor_limits(LEFT_MOTOR_PORT, 50, TURNING_SPEED)
    BP.set_motor_limits(RIGHT_MOTOR_PORT, 50, TURNING_SPEED)

    count = 0
    while (count < 4):
        count += 1
        forward(400)
        turnClockwise(-90)

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.