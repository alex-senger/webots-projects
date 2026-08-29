import math

import pytest

from epucklib import control, epuck


def test_wrap_angle_maps_into_pi_range():
    assert control.wrap_angle(0.0) == pytest.approx(0.0)
    assert control.wrap_angle(3 * math.pi) == pytest.approx(math.pi)
    assert control.wrap_angle(-3 * math.pi) == pytest.approx(math.pi)
    assert control.wrap_angle(1.5 * math.pi) == pytest.approx(-0.5 * math.pi)


def test_driving_straight_spins_both_wheels_equally():
    left, right = control.wheel_speeds(0.1, 0.0)
    assert left == pytest.approx(right)
    assert left == pytest.approx(0.1 / epuck.WHEEL_RADIUS_M)


def test_turning_in_place_spins_the_wheels_opposite():
    left, right = control.wheel_speeds(0.0, 1.0)
    assert left == pytest.approx(-right)
    # A positive omega turns left, so the right wheel runs forward.
    assert right > 0.0


def test_excessive_command_is_rescaled_but_keeps_its_curvature():
    v, omega = 5.0, 3.0  # far beyond what the motors can do
    left, right = control.wheel_speeds(v, omega)
    assert max(abs(left), abs(right)) == pytest.approx(epuck.MAX_WHEEL_SPEED)
    unclamped_left = (v - omega * epuck.AXLE_LENGTH_M / 2) / epuck.WHEEL_RADIUS_M
    unclamped_right = (v + omega * epuck.AXLE_LENGTH_M / 2) / epuck.WHEEL_RADIUS_M
    assert left / right == pytest.approx(unclamped_left / unclamped_right)


def test_go_to_point_drives_forward_when_aimed_at_the_target():
    v, omega = control.go_to_point(0.0, cruise_speed=0.09, k_heading=4.0, omega_max=4.0)
    assert v == pytest.approx(0.09)
    assert omega == pytest.approx(0.0)


def test_go_to_point_pivots_in_place_when_the_target_is_behind():
    v, omega = control.go_to_point(math.pi, cruise_speed=0.09, k_heading=4.0, omega_max=4.0)
    assert v == pytest.approx(0.0)
    assert abs(omega) == pytest.approx(4.0)


def test_go_to_point_clamps_omega_and_applies_the_slowdown():
    _, omega = control.go_to_point(1.0, cruise_speed=0.09, k_heading=4.0, omega_max=2.0)
    assert omega == pytest.approx(2.0)
    v, _ = control.go_to_point(0.0, cruise_speed=0.09, k_heading=4.0, omega_max=4.0, slowdown=0.25)
    assert v == pytest.approx(0.09 * 0.25)


# ---- blending the waypoint bearing with the repulsive field ----


def test_an_empty_field_steers_straight_at_the_target():
    error = control.blend_command(0.7, 0.0, 0.0, front=0.0, repulsion_gain=1.2)
    assert error == pytest.approx(0.7)


def test_a_field_from_the_left_pushes_the_command_to_the_right():
    # Target dead ahead, something pushing the robot toward -y.
    error = control.blend_command(0.0, 0.0, -0.5, front=0.0, repulsion_gain=1.2)
    assert error < 0.0


def test_the_gain_scales_how_hard_the_field_bends_the_command():
    gentle = control.blend_command(0.0, 0.0, -0.2, front=0.0, repulsion_gain=0.5)
    firm = control.blend_command(0.0, 0.0, -0.2, front=0.0, repulsion_gain=4.0)
    assert abs(firm) > abs(gentle)


def test_a_head_on_obstacle_breaks_the_tie_toward_the_side_the_target_is_on():
    # A purely backward push: attract and repel are collinear, so without the
    # tie-break the command would have no lateral component to steer on.
    for bearing, expected_sign in ((0.05, 1.0), (-0.05, -1.0)):
        error = control.blend_command(
            bearing, field_x=-1.0, field_y=0.0, front=0.9, repulsion_gain=1.2
        )
        assert math.copysign(1.0, error) == expected_sign
        # And it is a real commitment, not the 0.05 rad bearing on its own.
        assert abs(error) > 1.0


def test_the_tie_break_only_engages_once_something_is_really_close():
    # front must exceed 0.5; below that the command is the plain blend.
    plain = control.blend_command(0.05, -1.0, 0.0, front=0.0, repulsion_gain=1.2)
    still_plain = control.blend_command(0.05, -1.0, 0.0, front=0.4, repulsion_gain=1.2)
    nudged = control.blend_command(0.05, -1.0, 0.0, front=0.9, repulsion_gain=1.2)

    assert still_plain == pytest.approx(plain)
    # Untripped, a head-on push leaves the robot aimed almost straight back;
    # the tie-break swings it decisively sideways instead, so it goes round.
    assert abs(plain) > 2.8
    assert nudged == pytest.approx(1.92, abs=0.05)


def test_the_tie_break_leaves_an_already_lateral_command_alone():
    # command_y is well clear of the 0.1 dead zone, so the field is already
    # steering the robot round and needs no help.
    with_obstacle = control.blend_command(1.0, 0.0, 0.5, front=0.9, repulsion_gain=1.2)
    without = control.blend_command(1.0, 0.0, 0.5, front=0.0, repulsion_gain=1.2)
    assert with_obstacle == pytest.approx(without)


# ---- stall detection ----


def clear_ring():
    """A ring of IR readings with nothing in range."""
    return [epuck.IR_MAX_RANGE_M] * len(epuck.PS_NAMES)


def make_detector():
    return control.StallDetector(stall_speed_m=0.0002, stall_steps=5, escape_steps=3)


def test_the_detector_stays_quiet_until_the_threshold_is_passed():
    detector = make_detector()
    for _ in range(5):
        assert detector.update(0.0, clear_ring()) is None
    assert not detector.escaping
    # The sixth stalled step is the one that trips it.
    assert detector.update(0.0, clear_ring()) is not None
    assert detector.fired


def test_a_translating_robot_never_trips_the_detector():
    detector = make_detector()
    for _ in range(50):
        assert detector.update(0.01, clear_ring()) is None
    assert not detector.fired


def test_a_single_moving_step_resets_the_stall_count():
    detector = make_detector()
    for _ in range(5):
        detector.update(0.0, clear_ring())
    detector.update(0.01, clear_ring())  # it moved
    for _ in range(5):
        assert detector.update(0.0, clear_ring()) is None


def test_the_escape_runs_for_its_countdown_and_then_releases():
    detector = make_detector()
    for _ in range(6):
        detector.update(0.0, clear_ring())
    assert detector.fired and detector.escaping

    for _ in range(3):
        assert detector.update(0.0, clear_ring()) == pytest.approx(detector.turn)
        assert not detector.fired  # fired marks only the step it started on
    assert not detector.escaping
    assert detector.update(0.0, clear_ring()) is None


def test_the_escape_turns_toward_whichever_side_has_more_room():
    detector = make_detector()
    crowded_right = clear_ring()
    for index in epuck.RIGHT_SENSORS:
        crowded_right[index] = 0.005
    assert detector.escape_direction(crowded_right) > 0.0  # turn left

    crowded_left = clear_ring()
    for index in epuck.LEFT_SENSORS:
        crowded_left[index] = 0.005
    assert detector.escape_direction(crowded_left) < 0.0  # turn right
