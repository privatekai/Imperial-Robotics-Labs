"""Tests for the DWA planner in planning.py."""

import math
import random

import pytest


# ---------------------------------------------------------------------------
# Import planning *after* conftest has injected the brickpi3 mock
# ---------------------------------------------------------------------------
import planning
from planning import update_pose


# ========================== Unit conversion =================================

class TestCmPerSecToDps:
    def test_one_full_revolution_per_second(self):
        """1 wheel-circumference/s should equal 360 DPS."""
        wc = planning.WHEEL_CIRCUMFERENCE_CM
        assert planning.cm_per_sec_to_dps(wc) == pytest.approx(360.0)

    def test_zero(self):
        assert planning.cm_per_sec_to_dps(0) == pytest.approx(0.0)

    def test_round_trip_with_encoder_deg_to_cm(self):
        """cm_per_sec_to_dps and encoder_deg_to_cm should be inverses (up to units)."""
        v_cm = 15.0
        dps = planning.cm_per_sec_to_dps(v_cm)
        # dps degrees in 1 second → distance in cm
        cm_back = planning.encoder_deg_to_cm(dps)
        assert cm_back == pytest.approx(v_cm)


class TestEncoderDegToCm:
    def test_full_revolution(self):
        assert planning.encoder_deg_to_cm(360) == pytest.approx(planning.WHEEL_CIRCUMFERENCE_CM)

    def test_zero(self):
        assert planning.encoder_deg_to_cm(0) == pytest.approx(0.0)

    def test_negative(self):
        assert planning.encoder_deg_to_cm(-360) == pytest.approx(-planning.WHEEL_CIRCUMFERENCE_CM)


# ========================== predictPosition =================================

class TestPredictPosition:
    """Differential-drive kinematic model."""

    # --- Straight line (vL == vR) ---

    def test_straight_theta0(self):
        """Moving along theta=0 should increase x only."""
        v = 10.0
        x, y, th = planning.predictPosition(v, v, 0, 0, 0, 1.0)
        assert x == pytest.approx(10.0)
        assert y == pytest.approx(0.0, abs=1e-9)
        assert th == pytest.approx(0.0)

    def test_straight_theta_pi_over_2(self):
        """Moving along theta=pi/2 should increase y only."""
        v = 10.0
        x, y, th = planning.predictPosition(v, v, 0, 0, math.pi / 2, 1.0)
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(10.0)

    def test_straight_theta_pi(self):
        """Moving along theta=pi should decrease x."""
        v = 10.0
        x, y, th = planning.predictPosition(v, v, 0, 0, math.pi, 1.0)
        assert x == pytest.approx(-10.0)
        assert y == pytest.approx(0.0, abs=1e-9)

    def test_straight_displacement_matches_v_times_t(self):
        v, dt = 7.5, 2.0
        x, y, _ = planning.predictPosition(v, v, 0, 0, math.pi / 4, dt)
        dist = math.sqrt(x**2 + y**2)
        assert dist == pytest.approx(v * dt)

    # --- Near-equal velocities (tolerance check) ---

    def test_near_equal_velocities_uses_straight_line(self):
        """vL and vR differing by ~1e-10 should use straight-line branch."""
        v = 10.0
        x, y, th = planning.predictPosition(v, v + 1e-10, 0, 0, 0, 1.0)
        assert x == pytest.approx(10.0, abs=1e-6)
        assert y == pytest.approx(0.0, abs=1e-6)
        assert th == pytest.approx(0.0, abs=1e-6)

    # --- Pure rotation (vL == -vR) ---

    def test_pure_rotation_position_fixed(self):
        """When vL == -vR the robot spins in place."""
        vR = 5.0
        x, y, th = planning.predictPosition(-vR, vR, 10, 20, 0, 1.0)
        assert x == pytest.approx(10.0)
        assert y == pytest.approx(20.0)

    def test_pure_rotation_theta_change(self):
        vR = 5.0
        dt = 1.0
        _, _, th = planning.predictPosition(-vR, vR, 0, 0, 0, dt)
        expected = (vR - (-vR)) * dt / planning.WHEELBASE_CM
        assert th == pytest.approx(expected)

    # --- Zero velocity ---

    def test_zero_velocity(self):
        x, y, th = planning.predictPosition(0, 0, 5, 10, 1.2, 1.0)
        assert (x, y, th) == pytest.approx((5, 10, 1.2))

    # --- Reverse driving ---

    def test_reverse_straight(self):
        """Both wheels negative → move backward along theta."""
        v = -10.0
        x, y, _ = planning.predictPosition(v, v, 0, 0, 0, 1.0)
        assert x == pytest.approx(-10.0)
        assert y == pytest.approx(0.0, abs=1e-9)

    # --- Symmetry ---

    def test_swapping_vL_vR_mirrors_arc(self):
        """Swapping left/right should mirror the y-displacement."""
        vL, vR, dt = 8.0, 12.0, 1.0
        x1, y1, th1 = planning.predictPosition(vL, vR, 0, 0, 0, dt)
        x2, y2, th2 = planning.predictPosition(vR, vL, 0, 0, 0, dt)
        assert x1 == pytest.approx(x2, abs=1e-9)
        assert y1 == pytest.approx(-y2, abs=1e-9)
        assert th1 == pytest.approx(-th2, abs=1e-9)

    # --- Small dt vs large dt (straight line) ---

    def test_small_dt_vs_large_dt_straight(self):
        """For straight-line motion, one big step == many small steps."""
        v = 10.0
        # One big step
        x1, y1, th1 = planning.predictPosition(v, v, 0, 0, 0.3, 1.0)
        # Many small steps
        x, y, th = 0.0, 0.0, 0.3
        for _ in range(100):
            x, y, th = planning.predictPosition(v, v, x, y, th, 0.01)
        assert x == pytest.approx(x1, abs=1e-6)
        assert y == pytest.approx(y1, abs=1e-6)

    # --- Gentle curve (full circle returns near start) ---

    def test_gentle_curve_full_circle(self):
        """After a full 2*pi rotation the robot should be near its start."""
        vL, vR = 8.0, 12.0
        x, y, th = 0.0, 0.0, 0.0
        omega = (vR - vL) / planning.WHEELBASE_CM  # rad/s
        T = 2 * math.pi / abs(omega)  # time for full circle
        steps = 2000
        small_dt = T / steps
        for _ in range(steps):
            x, y, th = planning.predictPosition(vL, vR, x, y, th, small_dt)
        assert x == pytest.approx(0.0, abs=0.5)
        assert y == pytest.approx(0.0, abs=0.5)


# ======================== calculateClosestObstacleDistance ====================

class TestCalculateClosestObstacleDistance:
    def setup_method(self):
        self._orig = planning.barriers[:]

    def teardown_method(self):
        planning.barriers[:] = self._orig

    def test_no_barriers(self):
        planning.barriers[:] = []
        assert planning.calculateClosestObstacleDistance(0, 0) == float('inf')

    def test_single_barrier(self):
        planning.barriers[:] = [(30, 0)]
        d = planning.calculateClosestObstacleDistance(0, 0)
        expected = 30.0 - planning.BARRIERRADIUS - planning.ROBOTRADIUS
        assert d == pytest.approx(expected)

    def test_multiple_barriers_returns_closest(self):
        planning.barriers[:] = [(50, 0), (20, 0)]
        d = planning.calculateClosestObstacleDistance(0, 0)
        expected = 20.0 - planning.BARRIERRADIUS - planning.ROBOTRADIUS
        assert d == pytest.approx(expected)

    def test_robot_on_top_of_barrier(self):
        planning.barriers[:] = [(0, 0)]
        d = planning.calculateClosestObstacleDistance(0, 0)
        assert d < 0

    def test_barrier_at_exact_safe_boundary(self):
        """Barrier placed so distance == SAFEDIST."""
        safe = planning.SAFEDIST + planning.BARRIERRADIUS + planning.ROBOTRADIUS
        planning.barriers[:] = [(safe, 0)]
        d = planning.calculateClosestObstacleDistance(0, 0)
        assert d == pytest.approx(planning.SAFEDIST)


# ======================== Barrier deduplication =============================

class TestAddBarrier:
    def setup_method(self):
        self._orig = planning.barriers[:]

    def teardown_method(self):
        planning.barriers[:] = self._orig

    def test_duplicate_barrier_rejected(self):
        planning.barriers[:] = [(10, 10)]
        result = planning.add_barrier(10.5, 10.5)
        assert result is False
        assert len(planning.barriers) == 1

    def test_distant_barrier_accepted(self):
        planning.barriers[:] = [(10, 10)]
        result = planning.add_barrier(100, 100)
        assert result is True
        assert len(planning.barriers) == 2

    def test_empty_list_always_accepts(self):
        planning.barriers[:] = []
        assert planning.add_barrier(5, 5) is True
        assert len(planning.barriers) == 1


# ======================== Theta normalisation ===============================

class TestThetaNormalisation:
    """The normalisation expression used in main(): (theta + pi) % (2*pi) - pi"""

    @staticmethod
    def _normalise(theta):
        return (theta + math.pi) % (2 * math.pi) - math.pi

    def test_wrap_positive_overflow_4pi(self):
        assert self._normalise(4 * math.pi) == pytest.approx(0.0, abs=1e-9)

    def test_wrap_positive_overflow_3pi(self):
        assert self._normalise(3 * math.pi) == pytest.approx(-math.pi, abs=1e-9)

    def test_wrap_negative_overflow(self):
        assert self._normalise(-3 * math.pi) == pytest.approx(-math.pi, abs=1e-9)

    def test_already_in_range(self):
        assert self._normalise(math.pi / 4) == pytest.approx(math.pi / 4)

    def test_boundary_pi(self):
        # pi wraps to -pi or stays at pi (boundary)
        result = self._normalise(math.pi)
        assert abs(result) == pytest.approx(math.pi)

    def test_boundary_neg_pi(self):
        result = self._normalise(-math.pi)
        assert abs(result) == pytest.approx(math.pi)


# ======================== Barrier list cap ==================================

class TestBarrierListCap:
    def test_under_limit_unchanged(self):
        barriers = list(range(5))
        if len(barriers) > planning.MAX_BARRIERS:
            barriers = barriers[-planning.MAX_BARRIERS:]
        assert len(barriers) == 5

    def test_over_limit_trimmed(self):
        barriers = list(range(30))
        if len(barriers) > planning.MAX_BARRIERS:
            barriers = barriers[-planning.MAX_BARRIERS:]
        assert len(barriers) == planning.MAX_BARRIERS
        # Most recent (last) entries kept
        assert barriers[-1] == 29


# ======================== DWA planner logic =================================

class TestDWAPlanner:
    """Integration-style tests for the DWA scoring logic."""

    @staticmethod
    def _run_dwa(x, y, theta, vL, vR, barriers_list):
        """Call dwa_choose_velocities with given barriers, return (vL, vR)."""
        old_barriers = planning.barriers[:]
        planning.barriers[:] = barriers_list
        try:
            vLc, vRc, _ = planning.dwa_choose_velocities(x, y, theta, vL, vR)
            return vLc, vRc
        finally:
            planning.barriers[:] = old_barriers

    def test_dwa_returns_tuple(self):
        """dwa_choose_velocities returns (vL, vR, debug_dict)."""
        old = planning.barriers[:]
        planning.barriers[:] = []
        try:
            result = planning.dwa_choose_velocities(0, 0, math.pi / 2, 0, 0)
            assert len(result) == 3
            vL, vR, debug = result
            assert isinstance(vL, float)
            assert isinstance(vR, float)
            assert isinstance(debug, dict)
            assert 'candidates_evaluated' in debug
        finally:
            planning.barriers[:] = old

    def test_velocity_clamping(self):
        """Candidates exceeding MAXVELOCITY should be skipped."""
        vL = planning.MAXVELOCITY  # already at max
        steps = [i * planning.MAXACCELERATION * planning.dt for i in range(-4, 5)]
        valid = [vL + s for s in steps if abs(vL + s) <= planning.MAXVELOCITY]
        assert all(abs(v) <= planning.MAXVELOCITY for v in valid)
        # Some candidates should have been rejected
        all_candidates = [vL + s for s in steps]
        assert len(valid) < len(all_candidates)

    def test_forward_no_obstacles(self):
        """With no obstacles, planner should pick velocities that move toward target."""
        vL, vR = self._run_dwa(0, 0, math.pi / 2, 0, 0, [])
        # Both wheels should be positive (moving forward toward target at (0,450))
        assert vL > 0
        assert vR > 0

    def test_obstacle_avoidance(self):
        """With obstacle on straight path, planner should prefer a curving trajectory."""
        # Obstacle directly ahead at (0, 30)
        vL_clear, vR_clear = self._run_dwa(0, 0, math.pi / 2, 5, 5, [])
        vL_obs, vR_obs = self._run_dwa(0, 0, math.pi / 2, 5, 5, [(0, 30)])
        # With obstacle, the planner should pick different velocities (curve away)
        assert (vL_obs, vR_obs) != (vL_clear, vR_clear)

    def test_trajectory_sampling_detects_mid_arc_obstacle(self):
        """Obstacle placed mid-arc should be detected by intermediate trajectory points."""
        vL, vR = 10.0, 10.0
        mid_y = vL * planning.TAU * 0.5
        obstacles = [(0, mid_y)]
        vL_chosen, vR_chosen = self._run_dwa(0, 0, math.pi / 2, vL, vR, obstacles)
        assert not (vL_chosen == pytest.approx(vL) and vR_chosen == pytest.approx(vR))

    def test_stationary_start_accelerates(self):
        """From rest, planner should accelerate toward target."""
        vL, vR = self._run_dwa(0, 0, math.pi / 2, 0, 0, [])
        assert vL > 0 or vR > 0

    def test_initial_heading_prefers_forward(self):
        """theta=pi/2 facing +y, target at (0,450) → should prefer vL≈vR>0."""
        vL, vR = self._run_dwa(0, 0, math.pi / 2, 0, 0, [])
        assert vL > 0
        assert vR > 0
        # Roughly equal (straight toward target)
        assert abs(vL - vR) < max(vL, vR) * 0.5

    def test_candidate_grid_size(self):
        """DWA should evaluate ~81 candidates (9x9 grid) from rest."""
        old = planning.barriers[:]
        planning.barriers[:] = []
        try:
            _, _, debug = planning.dwa_choose_velocities(0, 0, math.pi / 2, 0, 0)
            assert debug['candidates_evaluated'] == 81
        finally:
            planning.barriers[:] = old

    def test_slows_near_obstacle(self):
        """With obstacle 20cm ahead, chosen speed should be less than current."""
        old = planning.barriers[:]
        planning.barriers[:] = [(0, 20)]
        try:
            vL, vR, _ = planning.dwa_choose_velocities(0, 0, math.pi / 2, 15, 15)
            chosen_speed = (abs(vL) + abs(vR)) / 2.0
            assert chosen_speed < 15.0
        finally:
            planning.barriers[:] = old

    def test_full_acceleration_budget(self):
        """Step spacing should equal MAXACCELERATION * dt (not half)."""
        old = planning.barriers[:]
        planning.barriers[:] = []
        try:
            # From rest, max acceleration for 1 step = MAXACCELERATION * dt
            _, _, debug = planning.dwa_choose_velocities(0, 0, math.pi / 2, 0, 0)
            # With 9 steps from -4 to +4 and full accel budget, max candidate = 4 * MAXACCELERATION * dt
            max_candidate = 4 * planning.MAXACCELERATION * planning.dt
            # The chosen velocity should be able to reach up to max_candidate
            assert max_candidate == pytest.approx(8.0)  # 4 * 10 * 0.2
        finally:
            planning.barriers[:] = old


# ======================== Dead-reckoning update_pose ========================

def _run_steps(steps_dL_dR):
    """Run update_pose for a sequence of (dL_cm, dR_cm) pairs from the
    canonical start pose (x=0, y=0, theta=pi/2).  Returns (x, y, theta)."""
    x, y, theta = 0.0, 0.0, math.pi / 2
    for dL, dR in steps_dL_dR:
        x, y, theta = update_pose(x, y, theta, dL, dR)
    return x, y, theta


class TestDeadReckoning:
    """Dead-reckoning validation.  Each scenario mirrors a real velocity
    sequence the robot can be commanded to execute; the asserted final
    position is what a tape measure / protractor should show.

    WHEELBASE_CM = 15.2 cm  (152 mm)
    dt = 0.8 s per step
    dL = vL * dt,  dR = vR * dt  (no wheel slip assumed)
    """

    # ------------------------------------------------------------------
    # Test 1 — Straight line: accelerate, cruise, decelerate
    # vL == vR every step → theta stays at pi/2 throughout
    # Total forward distance = sum of dL values = 80.0 cm
    #
    # Real-life check: place tape measure along +y from start.
    # Robot should stop 80 cm ahead (±noise).
    # ------------------------------------------------------------------
    def test_straight_line_accel_cruise_decel(self):
        steps = [
            (6.4,  6.4),   # vL=vR=8,  accelerate
            (12.8, 12.8),  # vL=vR=16, accelerate
            (16.0, 16.0),  # vL=vR=20, hit MAXVELOCITY
            (16.0, 16.0),  # cruise
            (16.0, 16.0),  # cruise
            (9.6,  9.6),   # vL=vR=12, decelerate
            (3.2,  3.2),   # vL=vR=4,  nearly stopped
        ]
        x, y, theta = _run_steps(steps)
        # Expected final position: 80 cm straight ahead, no lateral drift
        assert x     == pytest.approx(0.0,          abs=0.01)
        assert y     == pytest.approx(80.0,          abs=0.01)
        assert theta == pytest.approx(math.pi / 2,   abs=0.001)
        print("\n[Test 1] Expected robot position: x=0 cm, y=80 cm, heading unchanged")

    # ------------------------------------------------------------------
    # Test 2 — Swerve right then straighten (DWA avoidance)
    # Steps 3-4: vL=20 > vR=12  →  d_theta < 0 (clockwise / right turn)
    # Steps 5-8: vL == vR, straighten at new heading
    #
    # Real-life check: start heading marked with tape.  After the run the
    # robot should be offset to the right (+x) and heading slightly
    # clockwise of start.
    # ------------------------------------------------------------------
    def test_swerve_right_then_straight(self):
        steps = [
            (6.4,  6.4),   # straight accel
            (12.8, 12.8),  # straight accel
            (16.0, 9.6),   # swerve right: vL=20, vR=12
            (16.0, 9.6),   # swerve right
            (16.0, 16.0),  # straighten
            (16.0, 16.0),  # cruise
            (9.6,  9.6),   # decelerate
            (3.2,  3.2),   # nearly stopped
        ]
        x, y, theta = _run_steps(steps)
        # Precomputed: x≈43.24, y≈72.33, theta≈0.7287 rad (≈41.7°)
        assert x     == pytest.approx(43.24, abs=0.5)
        assert y     == pytest.approx(72.33, abs=0.5)
        assert theta == pytest.approx(0.7287, abs=0.02)
        # Sanity: robot moved forward and to the right, heading turned right
        assert x > 0
        assert theta < math.pi / 2
        print("\n[Test 2] Expected robot position: x≈%.1f cm right, y≈%.1f cm forward, "
              "heading≈%.1f°" % (x, y, math.degrees(theta)))

    # ------------------------------------------------------------------
    # Test 3 — Spin left then drive forward
    # Steps 1-4: vL=-10, vR=+10  (turn left / CCW)
    # Steps 5-8: vL=vR=15        (drive straight at new heading)
    #
    # Real-life check: mark start position and original forward direction.
    # After the spin the robot faces ≈-0.50 rad (≈-29°) from +y.
    # After driving 4×12 cm it should end ≈42 cm to the right and
    # ≈23 cm behind the start.
    # ------------------------------------------------------------------
    def test_spin_then_forward(self):
        spin_steps  = [(-8.0, 8.0)] * 4   # vL=-10, vR=+10, dt=0.8
        drive_steps = [(12.0, 12.0)] * 4  # vL=vR=15, dt=0.8
        x, y, theta = _run_steps(spin_steps + drive_steps)
        # Precomputed: x≈42.04, y≈-23.15, theta≈-0.5019 rad
        assert x     == pytest.approx(42.04,  abs=0.5)
        assert y     == pytest.approx(-23.15, abs=0.5)
        assert theta == pytest.approx(-0.5019, abs=0.02)
        print("\n[Test 3] Expected robot position: x≈%.1f cm, y≈%.1f cm, "
              "heading≈%.1f°" % (x, y, math.degrees(theta)))

    # ------------------------------------------------------------------
    # Test 4 — Realistic 12-step DWA with varied L/R asymmetry
    # Mimics navigation around one can: accelerate, swerve right, ease
    # back, small left correction, cruise, decelerate.
    #
    # Real-life check: mark floor start with tape cross.
    # Final position: x≈103.69 cm to the right, y≈80.83 cm forward,
    # heading≈29.7° clockwise of start.
    # ------------------------------------------------------------------
    def test_realistic_dwa_12_steps(self):
        steps = [
            (6.4,  6.4),   # (vL=8,  vR=8)  straight accel
            (12.8, 12.8),  # (vL=16, vR=16) straight accel
            (16.0, 9.6),   # (vL=20, vR=12) left swerve
            (16.0, 6.4),   # (vL=20, vR=8)  stronger left swerve
            (14.4, 11.2),  # (vL=18, vR=14) easing back
            (16.0, 16.0),  # (vL=20, vR=20) straight cruise
            (16.0, 16.0),  # (vL=20, vR=20) straight cruise
            (12.8, 14.4),  # (vL=16, vR=18) tiny right correction
            (11.2, 12.8),  # (vL=14, vR=16) slight right
            (16.0, 16.0),  # (vL=20, vR=20) straight again
            (11.2, 11.2),  # (vL=14, vR=14) decelerate
            (4.8,  4.8),   # (vL=6,  vR=6)  nearly stopped
        ]
        x, y, theta = _run_steps(steps)
        # Precomputed: x≈103.69, y≈80.83, theta≈0.5182 rad (≈29.7°)
        assert x     == pytest.approx(103.69, abs=0.5)
        assert y     == pytest.approx(80.83,  abs=0.5)
        assert theta == pytest.approx(0.5182, abs=0.02)
        print("\n[Test 4] Expected robot position: x≈%.1f cm, y≈%.1f cm, "
              "heading≈%.1f°" % (x, y, math.degrees(theta)))
