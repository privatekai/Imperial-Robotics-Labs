#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import brickpi3
import math
import time
from motion import Motion

# Calibration constants (update these after running calibration)
SCALE_CALIBRATED = 1.0127
OFFSET_CALIBRATED = 3.7975

# Sonar rotation motor
SONAR_MOTOR_SPEED   = 100          # degrees/sec
SONAR_MOTOR_TOL     = 5            # degrees, position tolerance
SONAR_LEFT_ANGLE    = -300         # motor degrees for 90° left  — CALIBRATE
SONAR_RIGHT_ANGLE   =  300         # motor degrees for 90° right — CALIBRATE

# Sonar head position in robot-local frame (cm, x=forward y=left) — CALIBRATE
FORWARD_OFFSET_X, FORWARD_OFFSET_Y = 0.0, 0.0
LEFT_OFFSET_X,    LEFT_OFFSET_Y    = 0.0, 0.0
RIGHT_OFFSET_X,   RIGHT_OFFSET_Y   = 0.0, 0.0


class SonarSensor:
    """
    Wrapper class for NXT ultrasonic sensor on BrickPi3.
    Applies calibration to account for sensor offset from robot axle center.
    Supports a rotation motor on PORT_C to face forward, left, or right.
    """

    def __init__(self, bp_instance, sensor_port=None, motor_port=None):
        """
        Initialize sonar sensor wrapper.

        Args:
            bp_instance: BrickPi3 instance
            sensor_port: Sensor port (defaults to BP.PORT_4)
            motor_port:  Motor port for sonar rotation (defaults to BP.PORT_C)
        """
        self.BP = bp_instance
        self.port = sensor_port if sensor_port is not None else bp_instance.PORT_2
        self.motor_port = motor_port if motor_port is not None else bp_instance.PORT_C

        # Configure sensor
        self.BP.set_sensor_type(self.port, bp_instance.SENSOR_TYPE.NXT_ULTRASONIC)

        # Configure rotation motor
        bp_instance.set_motor_limits(self.motor_port, power=50, speed=SONAR_MOTOR_SPEED)
        bp_instance.offset_motor_encoder(
            self.motor_port, bp_instance.get_motor_encoder(self.motor_port))

    def get_distance(self):
        """
        Get calibrated distance from sensor.

        Returns:
            float: Calibrated distance in cm, or None on sensor error
        """
        try:
            raw_distance = self.BP.get_sensor(self.port)
            calibrated_distance = SCALE_CALIBRATED * raw_distance + OFFSET_CALIBRATED
            return calibrated_distance
        except brickpi3.SensorError as error:
            print("Sonar sensor error:", error)
            return None

    def _wait_for_motor(self, target, timeout=10):
        """Poll encoder until within SONAR_MOTOR_TOL degrees of target, or timeout."""
        start = time.time()
        while time.time() - start < timeout:
            try:
                current = self.BP.get_motor_encoder(self.motor_port)
                if abs(target - current) < SONAR_MOTOR_TOL:
                    return True
                time.sleep(0.05)
            except IOError as e:
                print("IOError waiting for sonar motor:", e)
                time.sleep(0.1)
        print("WARNING: Sonar motor timeout (target=%d)" % target)
        return False

    def _rotate_to(self, direction):
        """
        Rotate sonar motor to face the given direction.

        Args:
            direction: 'forward', 'left', or 'right'
        """
        targets = {
            'forward': 0,
            'left':    SONAR_LEFT_ANGLE,
            'right':   SONAR_RIGHT_ANGLE,
        }
        if direction not in targets:
            raise ValueError("Unknown direction: %s" % direction)
        target = targets[direction]
        self.BP.set_motor_position(self.motor_port, target)
        self._wait_for_motor(target)

    def _offset_for(self, direction):
        """Return (ox, oy) in robot-local frame (x=forward, y=left) for the given direction."""
        if direction == 'forward':
            return FORWARD_OFFSET_X, FORWARD_OFFSET_Y
        elif direction == 'left':
            return LEFT_OFFSET_X, LEFT_OFFSET_Y
        elif direction == 'right':
            return RIGHT_OFFSET_X, RIGHT_OFFSET_Y
        else:
            raise ValueError("Unknown direction: %s" % direction)

    def get_distance_direction(self, direction):
        """Rotate sonar to direction, return calibrated distance (cm) or None."""
        self._rotate_to(direction)
        return self.get_distance()

    def get_directions(self):
        """Return (forward, left, right) calibrated distance triple (cm)."""
        return (
            self.get_distance_direction('forward'),
            self.get_distance_direction('left'),
            self.get_distance_direction('right'),
        )

    def ray(self, robot_x, robot_y, robot_theta_deg, direction):
        """
        Return (sx, sy, ray_angle_rad) — the sonar head world position and firing
        direction — for a particle at (robot_x, robot_y, robot_theta_deg).
        All sonar geometry (offsets, direction mapping) is private to this class.
        """
        ox, oy = self._offset_for(direction)
        angle = math.radians(robot_theta_deg)
        sx = robot_x + ox * math.cos(angle) - oy * math.sin(angle)
        sy = robot_y + ox * math.sin(angle) + oy * math.cos(angle)
        angle_offsets = {'forward': 0, 'left': math.pi / 2, 'right': -math.pi / 2}
        ray_angle = angle + angle_offsets[direction]
        return sx, sy, ray_angle


def calibrate_sensor(bp_instance, port=None):
    """
    Interactive calibration procedure for distance scale/offset.

    Prompts user to place robot at 20cm and 100cm from wall,
    calculates calibration constants, and prints values to
    manually update SCALE_CALIBRATED and OFFSET_CALIBRATED.

    Args:
        bp_instance: BrickPi3 instance
        port: Sensor port (defaults to BP.PORT_4)
    """
    sensor_port = port if port is not None else bp_instance.PORT_4
    bp_instance.set_sensor_type(sensor_port, bp_instance.SENSOR_TYPE.NXT_ULTRASONIC)

    print("=== Sonar Sensor Calibration ===")
    print()

    print("Place robot exactly 20cm from wall (measure from axle center)")
    input("Press Enter when ready...")

    readings_20cm = []
    for i in range(5):
        try:
            reading = bp_instance.get_sensor(sensor_port)
            readings_20cm.append(reading)
            print(f"  Reading {i+1}: {reading:.2f}cm")
        except brickpi3.SensorError as error:
            print(f"  Error reading sensor: {error}")
            return

    avg_20cm = sum(readings_20cm) / len(readings_20cm)
    print(f"Average at 20cm: {avg_20cm:.2f}cm")
    print()

    print("Place robot exactly 100cm from wall (measure from axle center)")
    input("Press Enter when ready...")

    readings_100cm = []
    for i in range(5):
        try:
            reading = bp_instance.get_sensor(sensor_port)
            readings_100cm.append(reading)
            print(f"  Reading {i+1}: {reading:.2f}cm")
        except brickpi3.SensorError as error:
            print(f"  Error reading sensor: {error}")
            return

    avg_100cm = sum(readings_100cm) / len(readings_100cm)
    print(f"Average at 100cm: {avg_100cm:.2f}cm")
    print()

    if avg_100cm == avg_20cm:
        print("Error: Sensor readings are identical at both distances!")
        print("Cannot calculate calibration. Please check sensor placement.")
        return

    scale = 80.0 / (avg_100cm - avg_20cm)
    offset = 20.0 - scale * avg_20cm

    scale = round(scale, 4)
    offset = round(offset, 4)

    print("Calibration complete!")
    print()
    print("Update sonarSensor.py with these values:")
    print(f"SCALE_CALIBRATED = {scale:.4f}")
    print(f"OFFSET_CALIBRATED = {offset:.4f}")
    print()
    print(f"Measured readings: 20cm={avg_20cm:.2f}cm, 100cm={avg_100cm:.2f}cm")


def calibrate_motor_angles(sonar):
    """
    Interactively determine SONAR_LEFT_ANGLE and SONAR_RIGHT_ANGLE by slowly
    driving the motor and reading the encoder when the user confirms position.
    """
    global SONAR_LEFT_ANGLE, SONAR_RIGHT_ANGLE

    print("\n=== Sonar Motor Angle Calibration ===")
    print("The sonar motor encoder is zeroed at the current (forward) position.")
    print()

    # --- Left angle ---
    print("Step 1: Calibrate LEFT angle")
    print("The motor will rotate slowly leftward.")
    print("Press Enter when the sonar is pointing 90° to the LEFT.")
    sonar.BP.set_motor_dps(sonar.motor_port, -30)
    input()
    sonar.BP.set_motor_dps(sonar.motor_port, 0)
    time.sleep(0.1)
    left_angle = sonar.BP.get_motor_encoder(sonar.motor_port)
    print(f"  Left encoder position: {left_angle}")

    # Return to centre
    print("  Returning to centre...")
    sonar.BP.set_motor_position(sonar.motor_port, 0)
    sonar._wait_for_motor(0)
    print()

    # --- Right angle ---
    print("Step 2: Calibrate RIGHT angle")
    print("The motor will rotate slowly rightward.")
    print("Press Enter when the sonar is pointing 90° to the RIGHT.")
    sonar.BP.set_motor_dps(sonar.motor_port, 30)
    input()
    sonar.BP.set_motor_dps(sonar.motor_port, 0)
    time.sleep(0.1)
    right_angle = sonar.BP.get_motor_encoder(sonar.motor_port)
    print(f"  Right encoder position: {right_angle}")

    # Return to centre
    print("  Returning to centre...")
    sonar.BP.set_motor_position(sonar.motor_port, 0)
    sonar._wait_for_motor(0)
    print()

    print("Update sonarSensor.py with these values:")
    print(f"SONAR_LEFT_ANGLE  = {left_angle}")
    print(f"SONAR_RIGHT_ANGLE = {right_angle}")
    print()

    if input("Apply for this session? (y/n): ").strip().lower() == "y":
        SONAR_LEFT_ANGLE = left_angle
        SONAR_RIGHT_ANGLE = right_angle
        print("Applied.\n")


def _read_sonar_avg(sonar, n=3):
    """Take n sonar readings and return the average, skipping None values."""
    readings = []
    for _ in range(n):
        d = sonar.get_distance()
        if d is not None:
            readings.append(d)
        time.sleep(0.1)
    if not readings:
        return None
    return sum(readings) / len(readings)


def calibrate_sonar_offset(sonar):
    """
    Interactively calibrate the sonar head's (x, y) offset from robot centre
    for a given direction, using two sonar readings at different headings.
    """
    global FORWARD_OFFSET_X, FORWARD_OFFSET_Y
    global LEFT_OFFSET_X,    LEFT_OFFSET_Y
    global RIGHT_OFFSET_X,   RIGHT_OFFSET_Y

    print("\n=== Sonar Head Offset Calibration ===")
    print("Directions: forward, left, right")
    direction = input("Enter direction to calibrate: ").strip().lower()
    if direction not in ('forward', 'left', 'right'):
        print("Invalid direction.")
        return

    # Setup instructions
    setup_msg = {
        'forward': "Position the robot so the sonar (facing FORWARD) points directly at a wall.",
        'left':    "Position the robot so the sonar (facing LEFT) points directly at a wall to the LEFT.",
        'right':   "Position the robot so the sonar (facing RIGHT) points directly at a wall to the RIGHT.",
    }
    print()
    print(setup_msg[direction])
    print("Measure D = perpendicular distance from ROBOT CENTRE to the wall (cm).")
    try:
        D = float(input("Enter D (cm): ").strip())
    except ValueError:
        print("Invalid input.")
        return

    # Rotate sonar to direction, take reading d1
    print(f"\nRotating sonar to '{direction}'...")
    sonar._rotate_to(direction)
    time.sleep(0.3)
    print("Taking reading d1...")
    d1 = _read_sonar_avg(sonar)
    if d1 is None:
        print("Sensor error — cannot calibrate.")
        return
    print(f"  d1 = {d1:.2f} cm")

    alpha_deg = 30.0
    alpha_rad = math.radians(alpha_deg)
    print(f"\nTurning robot {alpha_deg}° anticlockwise in-place...")
    motion_ctrl = Motion(sonar.BP)
    motion_ctrl.turnAntiClockwise(alpha_deg)   # no particles — calibration only
    time.sleep(0.2)

    # Re-point sonar at direction, take reading d2
    print(f"Re-pointing sonar to '{direction}'...")
    sonar._rotate_to(direction)
    time.sleep(0.3)
    print("Taking reading d2...")
    d2 = _read_sonar_avg(sonar)
    if d2 is None:
        print("Sensor error — cannot calibrate.")
        return
    print(f"  d2 = {d2:.2f} cm")

    # Compute offsets
    cos_a = math.cos(alpha_rad)
    sin_a = math.sin(alpha_rad)

    if direction == 'forward':
        ox = D - d1
        oy = ((d2 - d1) * cos_a - D * (1 - cos_a)) / sin_a
    elif direction == 'left':
        oy = D - d1
        ox = (D * (1 - cos_a) - (d2 - d1) * cos_a) / sin_a
    else:  # right
        oy = d1 - D
        ox = ((d2 - d1) * cos_a - D * (1 - cos_a)) / sin_a

    print(f"\nComputed offsets for '{direction}':")
    print(f"  ox = {ox:.4f} cm  (forward from robot centre)")
    print(f"  oy = {oy:.4f} cm  (left of robot centre; negative = right)")
    print()

    const_name = direction.upper()
    print("Update sonarSensor.py with these values:")
    print(f"{const_name}_OFFSET_X, {const_name}_OFFSET_Y = {ox:.4f}, {oy:.4f}")
    print()

    if input("Apply for this session? (y/n): ").strip().lower() == "y":
        if direction == 'forward':
            FORWARD_OFFSET_X, FORWARD_OFFSET_Y = ox, oy
        elif direction == 'left':
            LEFT_OFFSET_X, LEFT_OFFSET_Y = ox, oy
        else:
            RIGHT_OFFSET_X, RIGHT_OFFSET_Y = ox, oy
        print("Applied.\n")


if __name__ == "__main__":
    """
    Calibration menu when sonarSensor.py is executed directly.
    Usage: python3 sonarSensor.py
    """
    BP = brickpi3.BrickPi3()
    try:
        sonar = SonarSensor(BP)

        while True:
            print("\n=== Sonar Calibration Menu ===")
            print("1. Calibrate sensor (distance scale/offset)")
            print("2. Calibrate motor angles (left/right positions)")
            print("3. Calibrate sonar head offset (per direction)")
            print("4. Exit")
            choice = input("Choice: ").strip()

            if choice == "1":
                calibrate_sensor(BP, sonar.port)
            elif choice == "2":
                calibrate_motor_angles(sonar)
            elif choice == "3":
                calibrate_sonar_offset(sonar)
            elif choice == "4":
                break
            else:
                print("Invalid choice.")
    finally:
        BP.reset_all()
