"""Unicycle motion control for the differential-drive e-puck."""

import math
from collections.abc import Sequence

from epucklib.epuck import (
    AXLE_LENGTH_M,
    LEFT_SENSORS,
    MAX_WHEEL_SPEED,
    RIGHT_SENSORS,
    WHEEL_RADIUS_M,
)


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
    curvature. Therefore the robot still follows the arc it was asked for, just slower.
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


def blend_command(
    bearing: float,
    field_x: float,
    field_y: float,
    front: float,
    repulsion_gain: float,
) -> float:
    """Heading error that goes toward the target while avoiding what is near.

    The target bearing becomes a unit attract vector; the smoothed repulsive
    field is added with `repulsion_gain`, and the sum's direction is the
    heading to steer.

    A head-on obstacle cancels the field's tangential term, leaving attract and
    repel collinear with no reason to prefer either side. The tie-break spots
    that case and commits to whichever side the target is on.
    """
    attract_x, attract_y = math.cos(bearing), math.sin(bearing)

    command_x = attract_x + repulsion_gain * field_x
    command_y = attract_y + repulsion_gain * field_y

    if front > 0.5 and abs(command_y) < 0.1:
        command_y += math.copysign(0.5, bearing if bearing != 0.0 else 1.0)

    return math.atan2(command_y, command_x)


class StallDetector:
    """Notices that the robot has stopped moving, and runs an escape manoeuvre.

    `update` counts steps without translation and, past `stall_steps`, returns
    a turn rate to reverse with for the next `escape_steps` steps (None while
    driving normally). `fired` marks the step the escape began on which is the
    caller's cue to log it and discard whatever state led into the trap.
    """

    def __init__(
        self,
        stall_speed_m: float,
        stall_steps: int,
        escape_steps: int,
        turn_rate: float = 1.5,
    ) -> None:
        self.stall_speed_m = stall_speed_m
        self.stall_steps = stall_steps
        self.escape_steps = escape_steps
        self.turn_rate = turn_rate

        self.turn = 0.0
        self.fired = False
        self._stalled = 0
        self._remaining = 0

    @property
    def escaping(self) -> bool:
        """Is an escape manoeuvre still running?"""
        return self._remaining > 0

    def update(self, step_ds: float, distances: Sequence[float]) -> float | None:
        """Advance one control step"""
        self.fired = False

        if self._remaining > 0:
            self._remaining -= 1
            return self.turn

        if abs(step_ds) < self.stall_speed_m:
            self._stalled += 1
        else:
            self._stalled = 0

        if self._stalled <= self.stall_steps:
            return None

        self._stalled = 0
        self._remaining = self.escape_steps
        self.turn = self.escape_direction(distances)
        self.fired = True
        return self.turn

    def escape_direction(self, distances: Sequence[float]) -> float:
        """Turn toward whichever side has more room as the robot reverses."""
        left_room = sum(distances[index] for index in LEFT_SENSORS)
        right_room = sum(distances[index] for index in RIGHT_SENSORS)
        return self.turn_rate if left_room > right_room else -self.turn_rate
