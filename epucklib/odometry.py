"""Differential-drive kinematics and dead reckoning for the e-puck.

Everything here takes plain numbers, so it can be exercised without Webots.
"""

import math
from dataclasses import dataclass

from epucklib.control import wrap_angle
from epucklib.epuck import AXLE_LENGTH_M, WHEEL_RADIUS_M


@dataclass
class Pose:
    """A planar pose in the world frame: metres and radians."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


class DeadReckoning:
    """Tracks a pose from wheel-encoder positions.

    Encoder values are absolute wheel angles in radians, exactly what a Webots
    `PositionSensor` on a wheel reports. Integration happens at the arc
    midpoint, which is exact for constant-curvature motion between two samples
    and markedly better than the naive Euler form over long runs.
    """

    def __init__(self, left_rad: float, right_rad: float, pose: Pose | None = None) -> None:
        self._prev_left = left_rad
        self._prev_right = right_rad
        self.pose = pose if pose is not None else Pose()
        self.step_ds = 0.0

    def update(self, left_rad: float, right_rad: float) -> Pose:
        """Advance the pose by the encoder increment and return it."""
        d_left = (left_rad - self._prev_left) * WHEEL_RADIUS_M
        d_right = (right_rad - self._prev_right) * WHEEL_RADIUS_M
        self._prev_left = left_rad
        self._prev_right = right_rad

        ds = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / AXLE_LENGTH_M
        mid_theta = self.pose.theta + d_theta / 2.0

        self.pose = Pose(
            self.pose.x + ds * math.cos(mid_theta),
            self.pose.y + ds * math.sin(mid_theta),
            wrap_angle(self.pose.theta + d_theta),
        )
        self.step_ds = ds
        return self.pose


def wheel_speeds_to_twist(left_rad_s: float, right_rad_s: float) -> tuple[float, float]:
    """Convert wheel angular velocities (rad/s) to (linear m/s, angular rad/s)."""
    v_left = left_rad_s * WHEEL_RADIUS_M
    v_right = right_rad_s * WHEEL_RADIUS_M
    return (v_left + v_right) / 2.0, (v_right - v_left) / AXLE_LENGTH_M


def integrate_distance(left_rad_s: float, right_rad_s: float, dt_s: float) -> float:
    """Path length covered in dt_s.

    Absolute value, so reversing adds to the distance travelled rather than
    subtracting from it; a robot spinning on the spot covers no ground.
    """
    linear, _ = wheel_speeds_to_twist(left_rad_s, right_rad_s)
    return abs(linear) * dt_s
