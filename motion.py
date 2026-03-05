from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3

# UNITS ARE MILLIMETRES

MOVEMENT_SPEED = 1440  # Speed for moving forward (Degrees Per Second)
TURNING_SPEED = 300   # Speed for turning (Degrees Per Second)

PI = 3.14159627

WHEEL_DIAMETER = 67
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI
WHEELBASE_WIDTH = 152

DISTANCE_ERROR = -1.15 * PI # Making it bigger makes it go less far
ANGLE_ERROR = -3.074 # Making it bigger makes it turn more

MINI_WAIT_TIME = 0.05  # Time to wait after each movement (Seconds)

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

LEFT_MOTOR_PORT = BP.PORT_B
RIGHT_MOTOR_PORT = BP.PORT_C

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

def forward(distance: float):
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

        time.sleep(MINI_WAIT_TIME)  # Small pause after reaching target
        print("Forward movement completed\n")

    except IOError as error:
        print("IOError in forward: %s" % error)



def turnAntiClockwise(angle: float):
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

        time.sleep(MINI_WAIT_TIME)  # Small pause after reaching target
        print("Turn completed\n")

    except IOError as error:
        print("IOError in turnAntiClockwise: %s" % error)

def reset_motors():
    BP.reset_all()

if __name__ == "__main__":

    def calibrate_distance():
        global WHEEL_CIRCUMFERENCE, WHEEL_DIAMETER
        try:
            dist_mm = float(input("Commanded distance in mm (e.g. 1000): ").strip())
        except ValueError:
            print("Invalid input.")
            return

        print("Moving forward %g mm..." % dist_mm)
        forward(dist_mm)

        try:
            actual_mm = float(input("Measured actual distance traveled (mm): ").strip())
        except ValueError:
            print("Invalid input.")
            return

        # actual/commanded = (WC + DE) / (new_WC + DE)  =>  new_WC + DE = (WC + DE) * commanded/actual
        ratio = dist_mm / actual_mm
        new_wc = ratio * (WHEEL_CIRCUMFERENCE + DISTANCE_ERROR) - DISTANCE_ERROR
        new_wd = new_wc / PI

        print("\n  Old WHEEL_DIAMETER = %f  (circumference = %f)" % (WHEEL_DIAMETER, WHEEL_CIRCUMFERENCE))
        print("  New WHEEL_DIAMETER = %f  (circumference = %f)" % (new_wd, new_wc))
        print("  Update motion.py: WHEEL_DIAMETER = %f\n" % new_wd)

        if input("Apply for this session? (y/n): ").strip().lower() == "y":
            WHEEL_DIAMETER = new_wd
            WHEEL_CIRCUMFERENCE = new_wc
            print("Applied.\n")

    def calibrate_angle():
        global WHEELBASE_WIDTH
        try:
            angle_deg = float(input("Commanded angle in degrees (e.g. 360): ").strip())
        except ValueError:
            print("Invalid input.")
            return

        print("Turning %g degrees anticlockwise..." % angle_deg)
        turnAntiClockwise(angle_deg)

        try:
            actual_deg = float(input("Measured actual angle turned (degrees): ").strip())
        except ValueError:
            print("Invalid input.")
            return

        # actual/commanded = (WB + AE) / (new_WB + AE)  =>  new_WB + AE = (WB + AE) * commanded/actual
        ratio = angle_deg / actual_deg
        new_wb = ratio * (WHEELBASE_WIDTH + ANGLE_ERROR) - ANGLE_ERROR

        print("\n  Old WHEELBASE_WIDTH = %f" % WHEELBASE_WIDTH)
        print("  New WHEELBASE_WIDTH = %f" % new_wb)
        print("  Update motion.py: WHEELBASE_WIDTH = %f\n" % new_wb)

        if input("Apply for this session? (y/n): ").strip().lower() == "y":
            WHEELBASE_WIDTH = new_wb
            print("Applied.\n")

    # print("=== Motion Calibrator ===")
    # print("WHEEL_DIAMETER = %f  WHEELBASE_WIDTH = %f" % (WHEEL_DIAMETER, WHEELBASE_WIDTH))
    # print()

    # while True:
    #     print("1. Calibrate distance (WHEEL_DIAMETER / WHEEL_CIRCUMFERENCE)")
    #     print("2. Calibrate angle   (WHEELBASE_WIDTH)")
    #     print("3. Exit")
    #     choice = input("Choice: ").strip()

    #     if choice == "1":
    #         calibrate_distance()
    #     elif choice == "2":
    #         calibrate_angle()
    #     elif choice == "3":
    #         break
    #     else:
    #         print("Invalid choice.\n")

    forward(400)
    turnAntiClockwise(45)
    forward(200)

    BP.reset_all()
