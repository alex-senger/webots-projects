"""Forward kinematics of a 3-DoF planar (RRR) manipulator."""

from math import cos, degrees, radians, sin

LINKS_MM: tuple[float, float, float] = (
    30.0,
    25.0,
    20.0,
)  # Lengths in millimetres from the assignment brief


def _normalise_deg(angle_deg: float) -> float:
    """Reduce an angle to (-180, 180]."""
    return 180.0 - ((180.0 - angle_deg) % 360.0)


def _cumulative_angles(
    t1_deg: float, t2_deg: float, t3_deg: float
) -> tuple[float, float, float]:
    """Absolute orientation of each link, in radians, measured from +x."""
    t1 = radians(t1_deg)
    t12 = t1 + radians(t2_deg)
    t123 = t12 + radians(t3_deg)
    return t1, t12, t123


def forward_kinematics(
    t1_deg: float, t2_deg: float, t3_deg: float, links=LINKS_MM
) -> tuple[float, float, float]:
    """Return the end-effector pose (x_mm, y_mm, phi_deg).

    phi is the orientation of the final link: for a planar chain this is
    the sum of the joint angles, reduced to (-180, 180] because an
    orientation of 540 degrees and one of 180 degrees are the same pose.

    Non-finite angles are not validated: NaN propagates to a NaN pose, and
    an infinite angle raises ValueError from the trigonometric call.
    """
    a1, a2, a3 = links
    t1, t12, t123 = _cumulative_angles(t1_deg, t2_deg, t3_deg)
    x = a1 * cos(t1) + a2 * cos(t12) + a3 * cos(t123)
    y = a1 * sin(t1) + a2 * sin(t12) + a3 * sin(t123)
    return x, y, _normalise_deg(degrees(t123))


def joint_positions(t1_deg, t2_deg, t3_deg, links=LINKS_MM):
    """Return [base, joint2, joint3, end_effector] as (x_mm, y_mm) points."""
    a1, a2, a3 = links
    t1, t12, t123 = _cumulative_angles(t1_deg, t2_deg, t3_deg)
    x1, y1 = a1 * cos(t1), a1 * sin(t1)
    x2, y2 = x1 + a2 * cos(t12), y1 + a2 * sin(t12)
    x3, y3 = x2 + a3 * cos(t123), y2 + a3 * sin(t123)
    return [(0.0, 0.0), (x1, y1), (x2, y2), (x3, y3)]
