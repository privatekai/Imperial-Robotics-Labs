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

POSITION_TOLERANCE = 0.5  # Tolerance in degrees for position checking
TIMEOUT = 60  # Maximum time to wait for motors to reach position (Seconds)

def wait_for_motor_position(left_target, right_target):
    """
    Wait for motors to reach their target positions.
    Checks actual encoder positions instead of just waiting a fixed time.
    Returns True if targets reached, False if timeout occurred.
    """
    start_time = time.time()

    while time.time() - start_time < TIMEOUT:
        try:
            left_current = BP.get_motor_encoder(LEFT_MOTOR_PORT)
            right_current = BP.get_motor_encoder(RIGHT_MOTOR_PORT)

            left_error = abs(left_target - left_current)
            right_error = abs(right_target - right_current)

            # Check if both motors are within tolerance
            if left_error < POSITION_TOLERANCE and right_error < POSITION_TOLERANCE:
                print("Position reached: L=%d (target=%d, error=%d), R=%d (target=%d, error=%d)" %
                      (left_current, left_target, left_error, right_current, right_target, right_error))
                return True

            # Small sleep to avoid hammering the I2C bus
            time.sleep(0.05)

        except IOError as error:
            print(error)
            time.sleep(0.1)

    # Timeout occurred
    try:
        left_current = BP.get_motor_encoder(LEFT_MOTOR_PORT)
        right_current = BP.get_motor_encoder(RIGHT_MOTOR_PORT)
        print("WARNING: Timeout! Current: L=%d (target=%d), R=%d (target=%d)" %
              (left_current, left_target, right_current, right_target))
    except IOError as error:
        print(error)

    return False

def forward(distance: float):
    target = (360 * distance) / WHEEL_CIRCUMFERENCE

    try:
        # Reset both encoders to 0 to ensure synchronized absolute targets
        BP.offset_motor_encoder(LEFT_MOTOR_PORT, BP.get_motor_encoder(LEFT_MOTOR_PORT))
        BP.offset_motor_encoder(RIGHT_MOTOR_PORT, BP.get_motor_encoder(RIGHT_MOTOR_PORT))

        print("Moving forward %f mm\nTarget position: %f degrees" % (distance, target))

        # Set both motors simultaneously using combined ports
        # This sends a single command to both motors at exactly the same time
        BP.set_motor_position(LEFT_MOTOR_PORT + RIGHT_MOTOR_PORT, target)

        wait_for_motor_position(target, target)
        time.sleep(MINI_WAIT_TIME)  # Small pause after reaching target

    except IOError as error:
        print(error)


def turnClockwise(angle: float):
    offset = (WHEELBASE_WIDTH * PI * angle / 360) / WHEEL_CIRCUMFERENCE * 360

    try:
        # Reset both encoders to 0 for clean starting positions
        BP.offset_motor_encoder(LEFT_MOTOR_PORT, BP.get_motor_encoder(LEFT_MOTOR_PORT))
        BP.offset_motor_encoder(RIGHT_MOTOR_PORT, BP.get_motor_encoder(RIGHT_MOTOR_PORT))

        left_target = offset
        right_target = -offset

        print("Turning clockwise %f degrees\nA target: %f, D target: %f" %
              (angle, left_target, right_target))

        # Set both motor positions as close together as possible
        # Cannot use combined ports here because targets are different
        BP.set_motor_position(LEFT_MOTOR_PORT, left_target)
        BP.set_motor_position(RIGHT_MOTOR_PORT, right_target)

        wait_for_motor_position(left_target, right_target)
        time.sleep(MINI_WAIT_TIME)  # Small pause after reaching target

    except IOError as error:
        print(error)

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