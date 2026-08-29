import math

import pytest

from epucklib import epuck
from epucklib.odometry import DeadReckoning, Pose

R = epuck.WHEEL_RADIUS_M
L = epuck.AXLE_LENGTH_M


def test_a_fresh_pose_is_the_origin():
    pose = Pose()
    assert (pose.x, pose.y, pose.theta) == (0.0, 0.0, 0.0)


def test_equal_wheel_rotation_drives_straight_along_x():
    odom = DeadReckoning(0.0, 0.0)
    turn = 1.0 / R  # one metre's worth of wheel rotation
    pose = odom.update(turn, turn)
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(0.0)
    assert pose.theta == pytest.approx(0.0)


def test_opposite_wheel_rotation_turns_in_place():
    odom = DeadReckoning(0.0, 0.0)
    # Right wheel forward, left wheel back by the same amount: pure rotation.
    quarter = (math.pi / 2) * (L / 2) / R
    pose = odom.update(-quarter, quarter)
    assert pose.x == pytest.approx(0.0)
    assert pose.y == pytest.approx(0.0)
    assert pose.theta == pytest.approx(math.pi / 2)


def test_many_small_steps_trace_a_circle_back_to_the_start():
    odom = DeadReckoning(0.0, 0.0)
    # A full turn on the spot, in 360 increments, must return to theta ~ 0.
    step = (2 * math.pi / 360) * (L / 2) / R
    left = right = 0.0
    for _ in range(360):
        left -= step
        right += step
        odom.update(left, right)
    assert odom.pose.theta == pytest.approx(0.0, abs=1e-6)
    # Note: the wheels move symmetrically here, so ds is identically zero and
    # the pose never translates -- this position check holds for any scheme.
    # test_arc_integration_converges_quadratically_the_way_midpoint_does is
    # what actually pins the integration down.
    assert math.hypot(odom.pose.x, odom.pose.y) == pytest.approx(0.0, abs=1e-9)


def drive_arc(steps: int, sweep_rad: float, radius_m: float = 0.25) -> Pose:
    """Integrate a constant-curvature arc of `sweep_rad` in `steps` increments.

    Both wheels move forward, by different amounts: the arc is genuinely
    driven, not pivoted. Per step the centre advances `ds` and the heading
    turns `ds / radius_m`, which is what the two wheel increments below encode.
    """
    odom = DeadReckoning(0.0, 0.0)
    ds = radius_m * sweep_rad / steps
    d_left = ds * (1.0 - L / (2.0 * radius_m))
    d_right = ds * (1.0 + L / (2.0 * radius_m))

    left = right = 0.0
    for _ in range(steps):
        left += d_left / R
        right += d_right / R
        odom.update(left, right)
    return odom.pose


def test_a_driven_full_circle_comes_back_to_where_it_started():
    pose = drive_arc(steps=360, sweep_rad=2 * math.pi)
    assert math.hypot(pose.x, pose.y) == pytest.approx(0.0, abs=1e-9)
    assert pose.theta == pytest.approx(0.0, abs=1e-9)


def test_arc_integration_converges_quadratically_the_way_midpoint_does():
    """The regression guard that actually pins the integration scheme down.

    Neither closing a circle nor landing on the right heading discriminates
    between integration schemes: a full circle split into N equal steps is a
    regular N-gon under *any* of them (midpoint's chords point along
    dtheta/2, 3*dtheta/2, ...; Euler's along 0, dtheta, ...), and a closed
    set of equally spaced chords sums to zero either way. Both close to ~1e-15
    for any N, so an absolute position assertion on a full circle is passed by
    schemes with no trigonometry in them at all.

    What does discriminate is how fast the endpoint of a *partial* arc
    converges on the analytic one. Midpoint integration is second order, so
    halving the step size quarters the error: going from 36 steps to 360
    should shrink it about 100-fold. Euler is only first order and would
    manage about 10-fold. The ratio -- not the error itself -- is therefore
    the real assertion here, and 50x sits far enough above 10 to fail loudly
    if `DeadReckoning.update` ever stops evaluating the heading at the arc
    midpoint.
    """
    radius, sweep = 0.25, math.pi
    exact_x = radius * math.sin(sweep)
    exact_y = radius * (1.0 - math.cos(sweep))

    def endpoint_error(steps: int) -> float:
        pose = drive_arc(steps, sweep, radius)
        return math.hypot(pose.x - exact_x, pose.y - exact_y)

    coarse = endpoint_error(36)
    fine = endpoint_error(360)

    # Sanity: the coarse run really is discretisation-limited, not exact.
    assert coarse > 1e-6
    assert fine < coarse / 50.0


def test_step_ds_reports_the_distance_of_the_last_update_only():
    odom = DeadReckoning(0.0, 0.0)
    turn = 0.5 / R
    odom.update(turn, turn)
    assert odom.step_ds == pytest.approx(0.5)
    odom.update(2 * turn, 2 * turn)
    assert odom.step_ds == pytest.approx(0.5)


def test_a_starting_pose_can_be_supplied():
    odom = DeadReckoning(0.0, 0.0, Pose(1.0, 2.0, math.pi / 2))
    turn = 0.1 / R
    pose = odom.update(turn, turn)
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(2.1)
