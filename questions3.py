from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3
from visualisation import NUM_PARTICLES, ROBOT_START_POS, apply_all_forward, apply_all_turn, initial_drawing # import the BrickPi3 drivers
import numpy as np

# UNITS ARE MILLIMETRES

MOVEMENT_SPEED = 180  # Speed for moving forward (Degrees Per Second)
TURNING_SPEED = 150   # Speed for turning (Degrees Per Second)

PI = 3.14159627

WHEEL_DIAMETER = 67
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI
WHEELBASE_WIDTH = 152

DISTANCE_ERROR = -0.45 * PI # Making it bigger makes it go less far
ANGLE_ERROR = 0 # Making it bigger makes it turn more

MINI_WAIT_TIME = 0.75  # Time to wait after each movement (Seconds)

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

LEFT_MOTOR_PORT = BP.PORT_A
RIGHT_MOTOR_PORT = BP.PORT_D

POSITION_TOLERANCE = 5  # Tolerance in degrees for position checking (increased to prevent timeout issues)
TIMEOUT = 60  # Maximum time to wait for motors to reach position (Seconds)

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
            left_current = BP.get_motor_encoder(LEFT_MOTOR_PORT)
            right_current = BP.get_motor_encoder(RIGHT_MOTOR_PORT)

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
        left_current = BP.get_motor_encoder(LEFT_MOTOR_PORT)
        right_current = BP.get_motor_encoder(RIGHT_MOTOR_PORT)
        print("WARNING: Timeout after %d seconds! Current: L=%d (target=%.1f), R=%d (target=%.1f)" %
              (TIMEOUT, left_current, left_target, right_current, right_target))
    except IOError as error:
        print("IOError getting final positions: %s" % error)

    return False

def forward(particles, distance: float):
    target = (360 * distance) / (WHEEL_CIRCUMFERENCE + DISTANCE_ERROR)

    try:    # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
        # Reset both encoders to 0 to ensure synchronized absolute targets
        BP.offset_motor_encoder(LEFT_MOTOR_PORT, BP.get_motor_encoder(LEFT_MOTOR_PORT))
        BP.offset_motor_encoder(RIGHT_MOTOR_PORT, BP.get_motor_encoder(RIGHT_MOTOR_PORT))
        
        # Set speed limits for forward movement
        BP.set_motor_limits(LEFT_MOTOR_PORT, 50, MOVEMENT_SPEED)
        BP.set_motor_limits(RIGHT_MOTOR_PORT, 50, MOVEMENT_SPEED)

        print("Moving forward %f mm\nTarget position: %f degrees" % (distance, target))

        # Set both motors to the same target position
        BP.set_motor_position(LEFT_MOTOR_PORT, target)
        BP.set_motor_position(RIGHT_MOTOR_PORT, target)

        wait_for_motor_position(target, target)

        # Update particles
        particles = apply_all_forward(particles, distance)

        time.sleep(MINI_WAIT_TIME)  # Small pause after reaching target
        print("Forward movement completed\n")

        return particles

    except IOError as error:
        print("IOError in forward: %s" % error)



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
        BP.offset_motor_encoder(LEFT_MOTOR_PORT, BP.get_motor_encoder(LEFT_MOTOR_PORT))
        BP.offset_motor_encoder(RIGHT_MOTOR_PORT, BP.get_motor_encoder(RIGHT_MOTOR_PORT))

        # Set speed limits for turning
        BP.set_motor_limits(LEFT_MOTOR_PORT, 50, TURNING_SPEED)
        BP.set_motor_limits(RIGHT_MOTOR_PORT, 50, TURNING_SPEED)

        # Left wheel backward, right wheel forward (for anticlockwise turn)
        left_target = -offset
        right_target = offset

        print("Turning %f degrees anticlockwise\nLeft target: %f, Right target: %f" %
              (angle, left_target, right_target))

        # Set both motor positions (opposite directions for turning on the spot)
        BP.set_motor_position(LEFT_MOTOR_PORT, left_target)
        BP.set_motor_position(RIGHT_MOTOR_PORT, right_target)

        wait_for_motor_position(left_target, right_target)

        # Update particles
        particles = apply_all_turn(particles, angle)

        time.sleep(MINI_WAIT_TIME)  # Small pause after reaching target
        print("Turn completed\n")

        return particles

    except IOError as error:
        print("IOError in turnAntiClockwise: %s" % error)

if __name__ == "__main__":
    try:
        try:
            BP.offset_motor_encoder(LEFT_MOTOR_PORT, BP.get_motor_encoder(LEFT_MOTOR_PORT)) # reset encoder A
            BP.offset_motor_encoder(RIGHT_MOTOR_PORT, BP.get_motor_encoder(RIGHT_MOTOR_PORT)) # reset encoder D
        except IOError as error:
            print(error)
        
        # Initial motor limits (will be updated in forward() and turnClockwise())
        BP.set_motor_limits(LEFT_MOTOR_PORT, 50, MOVEMENT_SPEED)
        BP.set_motor_limits(RIGHT_MOTOR_PORT, 50, MOVEMENT_SPEED)

        particles = np.array([ROBOT_START_POS] * NUM_PARTICLES)
        weights = np.array([1/NUM_PARTICLES] * NUM_PARTICLES)
        
        initial_drawing(particles)

        time.sleep(1)

        for _ in range(4):
            for _ in range(4):
                particles = forward(particles, 100)
            particles = turnAntiClockwise(particles, 90)
        # particles = navigate_to_waypoint((80, 40), particles, weights)

    except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
        BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
