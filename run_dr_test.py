"""Dead-reckoning validation runner.

Run one of the four test sequences from test_planning.py on the real robot,
then measure the final position with a tape measure and compare.

Usage:
    python run_dr_test.py 1    # straight line → 80 cm ahead
    python run_dr_test.py 2    # swerve right  → x≈43 cm, y≈72 cm
    python run_dr_test.py 3    # spin + forward → x≈42 cm, y≈-23 cm
    python run_dr_test.py 4    # 12-step DWA    → x≈104 cm, y≈81 cm
"""

import sys
import time

from constants import (
    BP,
    LEFT_MOTOR_PORT as L,
    RIGHT_MOTOR_PORT as R,
    WHEEL_CIRCUMFERENCE_CM,
)


def cm_per_sec_to_dps(v):
    return (v / WHEEL_CIRCUMFERENCE_CM) * 360.0


# Each entry: (vL cm/s, vR cm/s)
TESTS = {
    1: {
        "name": "Straight line — accelerate, cruise, decelerate",
        "expected": "x = 0 cm,  y = 80 cm,  heading unchanged",
        "steps": [
            (8,  8),
            (16, 16),
            (20, 20),
            (20, 20),
            (20, 20),
            (12, 12),
            (4,  4),
        ],
    },
    2: {
        "name": "Swerve right then straight",
        "expected": "x ≈ 43 cm right,  y ≈ 72 cm forward,  heading ≈ 42° clockwise of start",
        "steps": [
            (8,  8),
            (16, 16),
            (20, 12),
            (20, 12),
            (20, 20),
            (20, 20),
            (12, 12),
            (4,  4),
        ],
    },
    3: {
        "name": "Spin left then drive forward",
        "expected": "x ≈ 42 cm right,  y ≈ -23 cm (behind start),  heading ≈ -29° of start",
        "steps": [
            (-10, 10),
            (-10, 10),
            (-10, 10),
            (-10, 10),
            (15, 15),
            (15, 15),
            (15, 15),
            (15, 15),
        ],
    },
    4: {
        "name": "12-step DWA sequence",
        "expected": "x ≈ 104 cm right,  y ≈ 81 cm forward,  heading ≈ 30° clockwise of start",
        "steps": [
            (8,  8),
            (16, 16),
            (20, 12),
            (20,  8),
            (18, 14),
            (20, 20),
            (20, 20),
            (16, 18),
            (14, 16),
            (20, 20),
            (14, 14),
            (6,  6),
        ],
    },
}

DT = 0.8  # seconds per step


def run(test_num):
    t = TESTS[test_num]
    print("\n=== Test %d: %s ===" % (test_num, t["name"]))
    print("Expected final position: %s" % t["expected"])
    print("\nSteps (vL, vR) in cm/s, %.1f s each:" % DT)
    for i, (vL, vR) in enumerate(t["steps"], 1):
        print("  %2d: vL=%+5.1f  vR=%+5.1f  →  dL=%+.1f cm  dR=%+.1f cm" %
              (i, vL, vR, vL * DT, vR * DT))

    print("\nPlace tape marks at the start position and mark the forward direction.")
    input("Press Enter when ready to run...")

    BP.offset_motor_encoder(L, BP.get_motor_encoder(L))
    BP.offset_motor_encoder(R, BP.get_motor_encoder(R))

    try:
        for i, (vL, vR) in enumerate(t["steps"], 1):
            print("  step %d: vL=%+.1f  vR=%+.1f" % (i, vL, vR))
            BP.set_motor_dps(L, cm_per_sec_to_dps(vL))
            BP.set_motor_dps(R, cm_per_sec_to_dps(vR))
            time.sleep(DT)
    finally:
        BP.set_motor_dps(L, 0)
        BP.set_motor_dps(R, 0)
        BP.reset_all()

    print("\nDone. Measure and compare:")
    print("  Expected: %s" % t["expected"])


if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1] not in ("1", "2", "3", "4"):
        print("Usage: python run_dr_test.py <1|2|3|4>")
        sys.exit(1)
    run(int(sys.argv[1]))
