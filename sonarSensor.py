#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
import time

import brickpi3

# Calibration constants (update these after running calibration)
SCALE_CALIBRATED = 1.0256
OFFSET_CALIBRATED = 2.5641


class SonarSensor:
    """
    Wrapper class for NXT ultrasonic sensor on BrickPi3.
    Applies calibration to account for sensor offset from robot axle center.
    """

    def __init__(self, bp_instance, port=None):
        """
        Initialize sonar sensor wrapper.

        Args:
            bp_instance: BrickPi3 instance
            port: Sensor port (defaults to BP.PORT_4)
        """
        self.BP = bp_instance
        self.port = port if port is not None else bp_instance.PORT_2

        # Configure sensor
        self.BP.set_sensor_type(self.port, bp_instance.SENSOR_TYPE.NXT_ULTRASONIC)

        time.sleep(1)

    def get_distance(self):
        """
        Get calibrated distance from sensor.

        Returns:
            float: Calibrated distance in cm, or None on sensor error
        """
        try:
            # Read raw distance from sensor
            raw_distance = self.BP.get_sensor(self.port)

            # Apply calibration
            calibrated_distance = SCALE_CALIBRATED * raw_distance + OFFSET_CALIBRATED

            return calibrated_distance

        except brickpi3.SensorError as error:
            print("Sonar sensor error:", error)
            return None


def calibrate_sensor(bp_instance, port=None):
    """
    Interactive calibration procedure.

    Prompts user to place robot at 20cm and 100cm from wall,
    calculates calibration constants, and prints values to
    manually update SCALE_CALIBRATED and OFFSET_CALIBRATED.

    Args:
        bp_instance: BrickPi3 instance
        port: Sensor port (defaults to BP.PORT_4)
    """
    # Configure sensor
    sensor_port = port if port is not None else bp_instance.PORT_2
    bp_instance.set_sensor_type(sensor_port, bp_instance.SENSOR_TYPE.NXT_ULTRASONIC)

    print("=== Sonar Sensor Calibration ===")
    print()

    # First measurement at 20cm
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

    # Second measurement at 100cm
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

    # Calculate calibration constants
    if avg_100cm == avg_20cm:
        print("Error: Sensor readings are identical at both distances!")
        print("Cannot calculate calibration. Please check sensor placement.")
        return

    # Linear calibration: actual = scale * measured + offset
    # We have two points: (avg_20cm, 20.0) and (avg_100cm, 100.0)
    scale = 80.0 / (avg_100cm - avg_20cm)
    offset = 20.0 - scale * avg_20cm

    # Round to 4 decimal places
    scale = round(scale, 4)
    offset = round(offset, 4)

    # Print results
    print("Calibration complete!")
    print()
    print("Update sonarSensor.py with these values:")
    print(f"SCALE_CALIBRATED = {scale:.4f}")
    print(f"OFFSET_CALIBRATED = {offset:.4f}")
    print()
    print(f"Measured readings: 20cm={avg_20cm:.2f}cm, 100cm={avg_100cm:.2f}cm")


if __name__ == "__main__":
    """
    Run calibration when sonarSensor.py is executed directly.
    Usage: python3 sonarSensor.py
    """
    BP = brickpi3.BrickPi3()
    try:
        calibrate_sensor(BP)
    finally:
        BP.reset_all()
