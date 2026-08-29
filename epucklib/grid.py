"""An occupancy and coverage grid over a square arena.

Two layers share one geometry: `covered` records where the robot's body has
already passed, `state` records what is known about each cell. Indexing is
[row, col] with row following +y and col following +x, so the arrays can be
handed straight to `imshow(origin="lower")` for plotting.
"""

import math

import numpy as np

from epucklib.epuck import ARENA_HALF_M, BODY_RADIUS_M

UNKNOWN = 0
FREE = 1
OCCUPIED = 2


def _disk_offsets(radius_m: float, cell_size_m: float) -> list[tuple[int, int]]:
    """Cell offsets whose centres lie within `radius_m` of the middle cell."""
    reach = int(math.floor(radius_m / cell_size_m)) + 1
    offsets = []
    for d_row in range(-reach, reach + 1):
        for d_col in range(-reach, reach + 1):
            if math.hypot(d_row, d_col) * cell_size_m <= radius_m:
                offsets.append((d_row, d_col))
    return offsets


class CoverageMap:
    """What the robot knows about the floor, and how much of it it has swept."""

    def __init__(
        self,
        half_extent_m: float = ARENA_HALF_M,
        cell_size_m: float = 0.02,
        robot_radius_m: float = BODY_RADIUS_M,
        hits_to_occupy: int = 2,
    ) -> None:
        self.half = half_extent_m
        self.cell = cell_size_m
        self.robot_radius = robot_radius_m
        self.n = int(round(2.0 * half_extent_m / cell_size_m))

        self.covered = np.zeros((self.n, self.n), dtype=bool)
        self.state = np.full((self.n, self.n), UNKNOWN, dtype=np.int8)

        # An obstacle must be seen twice before it is believed, which throws
        # away the occasional spurious IR spike without needing a full
        # log-odds model.
        self._hits = np.zeros((self.n, self.n), dtype=np.int16)
        self._hits_to_occupy = hits_to_occupy

        self._disk = _disk_offsets(robot_radius_m, cell_size_m)

    # ---- geometry ----

    def world_to_cell(self, x: float, y: float):
        """(row, col) containing a world point, or None if it is off the map."""
        col = int(math.floor((x + self.half) / self.cell))
        row = int(math.floor((y + self.half) / self.cell))
        if 0 <= row < self.n and 0 <= col < self.n:
            return row, col
        return None

    def cell_center(self, row: int, col: int) -> tuple[float, float]:
        """World coordinates of the middle of a cell."""
        return (
            (col + 0.5) * self.cell - self.half,
            (row + 0.5) * self.cell - self.half,
        )

    # ---- recording ----

    def stamp_covered(self, x: float, y: float) -> None:
        """Mark the robot's footprint at (x, y) as covered, and as free space.

        The robot is standing there, so those cells demonstrably hold no
        obstacle -- but a cell already believed occupied is never downgraded,
        because a grazing footprint should not erase a box.
        """
        center = self.world_to_cell(x, y)
        if center is None:
            return
        row0, col0 = center
        for d_row, d_col in self._disk:
            row, col = row0 + d_row, col0 + d_col
            if 0 <= row < self.n and 0 <= col < self.n:
                self.covered[row, col] = True
                if self.state[row, col] == UNKNOWN:
                    self.state[row, col] = FREE

    # ---- reporting ----

    def coverage_counts(self) -> tuple[int, int]:
        """(cells covered, cells that could be covered at all)."""
        coverable = self.state != OCCUPIED
        return int((self.covered & coverable).sum()), int(coverable.sum())

    def coverage_fraction(self) -> float:
        """Covered share of the coverable floor, in [0, 1]."""
        covered, coverable = self.coverage_counts()
        return covered / coverable if coverable else 0.0
