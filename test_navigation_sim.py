#!/usr/bin/env python3
"""
Simulation-only test of waypoint navigation.
Tests the navigation logic and particle updates without physical robot.
"""
import numpy as np
from visualisation import (NUM_PARTICLES, ROBOT_START_POS, robot_position,
                          apply_all_forward, apply_all_turn, print_formatted_robot_pos)

def simulate_navigate_to_waypoint(waypoint, particles, weights):
    """
    Simulated version of navigate_to_waypoint - no physical robot needed.
    """
    # Get current position from particles
    robot_x, robot_y, robot_facing = robot_position(particles, weights)
    w_x, w_y = waypoint

    print(f"\n  Current: ({robot_x:.1f}, {robot_y:.1f}), facing {robot_facing:.1f}°")
    print(f"  Target:  ({w_x}, {w_y})")

    # Calculate distance and angle to target
    distance = np.sqrt((w_x - robot_x)**2 + (w_y - robot_y)**2)

    # Calculate target facing angle
    dx = w_x - robot_x
    dy = w_y - robot_y

    if dx == 0:
        facing_target = 90 if dy > 0 else -90
    else:
        facing_target = np.rad2deg(np.arctan(dy / dx))
        if dx < 0:
            facing_target += 180

    # Calculate turn needed (anticlockwise convention: positive = left turn)
    turn_angle = facing_target - robot_facing

    print(f"  Distance: {distance:.1f}mm")
    print(f"  Turn: {turn_angle:.1f}° (target facing: {facing_target:.1f}°)")

    # Simulate turn (anticlockwise for positive angles)
    particles = apply_all_turn(particles, turn_angle)

    # Simulate forward movement
    particles = apply_all_forward(particles, distance)

    return particles

def run_simulation():
    """Run simulation tests of waypoint navigation."""
    print("="*70)
    print("WAYPOINT NAVIGATION SIMULATION TEST")
    print("="*70)
    print("\nCoordinate system: 0°=right, 90°=up, positive angles=anticlockwise")
    print("Starting position: (0, 0), facing 0° (right)\n")

    # Initialize particles and weights
    particles = np.array([ROBOT_START_POS] * NUM_PARTICLES)
    weights = np.array([1/NUM_PARTICLES] * NUM_PARTICLES)

    test_cases = [
        {
            "name": "BUG FIX TEST",
            "description": "The reported bug: should reach (400,400) not (400,-400)",
            "waypoints": [(400, 0), (400, 400)],
            "expected": [
                {"pos": (400, 0), "facing": 0},
                {"pos": (400, 400), "facing": 90}
            ]
        },
        {
            "name": "ANTI-CLOCKWISE SQUARE",
            "description": "Complete square with left turns",
            "waypoints": [(100, 0), (100, 100), (0, 100), (0, 0)],
            "expected": [
                {"pos": (100, 0), "facing": 0},
                {"pos": (100, 100), "facing": 90},
                {"pos": (0, 100), "facing": 180},
                {"pos": (0, 0), "facing": -90}  # or 270
            ]
        },
        {
            "name": "DIAGONAL MOVEMENT",
            "description": "Test diagonal navigation",
            "waypoints": [(200, 200), (0, 0)],
            "expected": [
                {"pos": (200, 200), "facing": 45},
                {"pos": (0, 0), "facing": -135}  # or 225
            ]
        }
    ]

    for test_idx, test in enumerate(test_cases, 1):
        print("\n" + "="*70)
        print(f"TEST {test_idx}: {test['name']}")
        print(f"Description: {test['description']}")
        print("="*70)

        # Reset to origin for each test
        particles = np.array([ROBOT_START_POS] * NUM_PARTICLES)

        for wp_idx, waypoint in enumerate(test['waypoints']):
            print(f"\n→ Waypoint {wp_idx + 1}: {waypoint}")

            # Navigate
            particles = simulate_navigate_to_waypoint(waypoint, particles, weights)

            # Check result
            robot_x, robot_y, robot_facing = robot_position(particles, weights)
            expected = test['expected'][wp_idx]
            exp_x, exp_y = expected['pos']
            exp_facing = expected['facing']

            # Calculate errors
            pos_error = np.sqrt((robot_x - exp_x)**2 + (robot_y - exp_y)**2)

            # Normalize angles to [-180, 180] for comparison
            angle_diff = ((robot_facing - exp_facing + 180) % 360) - 180

            print(f"\n  Result: ({robot_x:.1f}, {robot_y:.1f}), facing {robot_facing:.1f}°")
            print(f"  Expected: ({exp_x}, {exp_y}), facing {exp_facing}°")

            # Check if within tolerance (allow for particle noise)
            if pos_error < 5.0 and abs(angle_diff) < 5:
                print(f"  ✓ PASS (pos error: {pos_error:.2f}mm, angle error: {angle_diff:.2f}°)")
            else:
                print(f"  ✗ FAIL (pos error: {pos_error:.2f}mm, angle error: {angle_diff:.2f}°)")

    print("\n" + "="*70)
    print("SIMULATION COMPLETE")
    print("="*70)

if __name__ == "__main__":
    run_simulation()
