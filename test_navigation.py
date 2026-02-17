import time
import numpy as np
from questions3 import BP, LEFT_MOTOR_PORT, MOVEMENT_SPEED, RIGHT_MOTOR_PORT
from navigation import navigate_to_waypoint
from visualisation import NUM_PARTICLES, ROBOT_START_POS, initial_drawing, robot_position

def test_waypoint_navigation():
    """
    Test the waypoint navigation with the bug example and a square pattern.
    This verifies both the physical robot movement and particle display.
    """
    try:
        # Reset encoders
        try:
            BP.offset_motor_encoder(LEFT_MOTOR_PORT, BP.get_motor_encoder(LEFT_MOTOR_PORT))
            BP.offset_motor_encoder(RIGHT_MOTOR_PORT, BP.get_motor_encoder(RIGHT_MOTOR_PORT))
        except IOError as error:
            print(error)

        # Initial motor limits
        BP.set_motor_limits(LEFT_MOTOR_PORT, 50, MOVEMENT_SPEED)
        BP.set_motor_limits(RIGHT_MOTOR_PORT, 50, MOVEMENT_SPEED)

        # Initialize particles and weights
        particles = np.array([ROBOT_START_POS] * NUM_PARTICLES)
        weights = np.array([1/NUM_PARTICLES] * NUM_PARTICLES)

        # Draw initial state
        print("="*60)
        print("INITIALIZING")
        print("="*60)
        initial_drawing(particles)
        time.sleep(2)

        # Define test waypoints
        test_cases = [
            {
                "name": "Bug Test Case",
                "description": "Tests the (400,0) → (400,400) bug",
                "waypoints": [(400, 0), (400, 400)]
            },
            {
                "name": "Return to Origin",
                "description": "Return to starting position",
                "waypoints": [(400, 0), (0, 0)]
            },
            {
                "name": "Anti-clockwise Square",
                "description": "Navigate a square pattern",
                "waypoints": [(100, 0), (100, 100), (0, 100), (0, 0)]
            }
        ]

        for test in test_cases:
            print("\n" + "="*60)
            print(f"TEST: {test['name']}")
            print(f"Description: {test['description']}")
            print("="*60)

            for i, waypoint in enumerate(test['waypoints'], 1):
                print(f"\n--- Waypoint {i}/{len(test['waypoints'])}: {waypoint} ---")

                # Show position before movement
                robot_x, robot_y, robot_facing = robot_position(particles, weights)
                print(f"BEFORE: Position=({robot_x:.1f}, {robot_y:.1f}), Facing={robot_facing:.1f}°")

                # Navigate to waypoint
                particles = navigate_to_waypoint(waypoint, particles, weights)

                # Show position after movement
                robot_x, robot_y, robot_facing = robot_position(particles, weights)
                print(f"AFTER:  Position=({robot_x:.1f}, {robot_y:.1f}), Facing={robot_facing:.1f}°")

                # Verify we reached the target (within tolerance)
                target_x, target_y = waypoint
                distance_error = np.sqrt((robot_x - target_x)**2 + (robot_y - target_y)**2)

                if distance_error < 20:  # 20mm tolerance
                    print(f"✓ SUCCESS: Reached waypoint (error: {distance_error:.1f}mm)")
                else:
                    print(f"✗ FAILED: Did not reach waypoint (error: {distance_error:.1f}mm)")

                time.sleep(1)

            # Pause between test cases
            print(f"\nTest '{test['name']}' completed. Waiting 3 seconds...")
            time.sleep(3)

        print("\n" + "="*60)
        print("ALL TESTS COMPLETED")
        print("="*60)

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    finally:
        BP.reset_all()

if __name__ == "__main__":
    print("""
╔════════════════════════════════════════════════════════════╗
║         WAYPOINT NAVIGATION TEST PROGRAM                   ║
╚════════════════════════════════════════════════════════════╝

This program will test:
1. The bug fix: (400,0) → (400,400)
   - Should turn anticlockwise 90° and move up
   - Should end at (400, 400) NOT (400, -400)

2. Return to origin: (400,0) → (0,0)
   - Should turn and move back to start

3. Anti-clockwise square pattern
   - Should complete a square with left turns

The particle display code will show positions throughout.
Press Ctrl+C to stop at any time.
""")

    input("Press ENTER to start the test...")
    test_waypoint_navigation()
