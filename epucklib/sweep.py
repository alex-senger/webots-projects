"""Boustrophedon ("as the ox ploughs") coverage lanes.

The classic systematic sweep: parallel lanes driven in alternating directions,
so the robot ends each lane next to the start of the next one.
"""

import math


def lane_waypoints(limit_m: float, spacing_m: float) -> list[tuple[float, float]]:
    """Serpentine waypoints covering the square [-limit_m, limit_m].

    Lanes run parallel to the y axis and step along x. The lane count is
    rounded up so the true spacing is never wider than asked for, and the
    outermost lanes land exactly on the limits. Otherwise the strips along
    the walls would be left for the gap-filling phase to mop up.
    """
    span = 2.0 * limit_m
    lanes = max(2, int(math.ceil(span / spacing_m)) + 1)
    step = span / (lanes - 1)

    points: list[tuple[float, float]] = []
    for index in range(lanes):
        x = -limit_m + index * step
        start_y, end_y = (-limit_m, limit_m) if index % 2 == 0 else (limit_m, -limit_m)
        points.append((x, start_y))
        points.append((x, end_y))
    return points
