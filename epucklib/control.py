"""Unicycle motion control for the differential-drive e-puck."""

import math

from epucklib.epuck import AXLE_LENGTH_M, MAX_WHEEL_SPEED, WHEEL_RADIUS_M


def wrap_angle(angle: float) -> float:
    """Map an angle to (-pi, pi]."""
    wrapped = math.atan2(math.sin(angle), math.cos(angle))
    # Handle the edge case where atan2 returns -π (should be in the range (-π, π])
    if wrapped < 0 and wrapped <= -math.pi + 1e-10:
        return math.pi
    return wrapped


def wheel_speeds(v: float, omega: float) -> tuple[float, float]:
    """Convert a unicycle command to (left, right) wheel speeds in rad/s.

    If either wheel would exceed the motor limit both are rescaled by the same
    factor, which saturates the speed without distorting the commanded
    curvature -- the robot still follows the arc it was asked for, just slower.
    """
    left = (v - omega * AXLE_LENGTH_M / 2.0) / WHEEL_RADIUS_M
    right = (v + omega * AXLE_LENGTH_M / 2.0) / WHEEL_RADIUS_M
    fastest = max(abs(left), abs(right))
    if fastest > MAX_WHEEL_SPEED:
        scale = MAX_WHEEL_SPEED / fastest
        left *= scale
        right *= scale
    return left, right


def go_to_point(
    heading_error: float,
    cruise_speed: float,
    k_heading: float,
    omega_max: float,
    slowdown: float = 1.0,
) -> tuple[float, float]:
    """Proportional heading law toward a target bearing.

    Forward speed is scaled by cos(error), so the robot pivots on the spot when
    the target lies behind it and only commits to full speed once it is aimed
    the right way; `slowdown` in [0, 1] additionally brakes near obstacles.
    """
    v = cruise_speed * max(math.cos(heading_error), 0.0) * slowdown
    omega = max(-omega_max, min(omega_max, k_heading * heading_error))
    return v, omega
