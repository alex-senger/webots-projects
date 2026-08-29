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
