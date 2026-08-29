"""An occupancy and coverage grid over a square arena.

Two layers share one geometry: `covered` records where the robot's body has
already passed, `state` records what is known about each cell. Indexing is
[row, col] with row following +y and col following +x, so the arrays can be
handed straight to `imshow(origin="lower")` for plotting.
"""

import math
from collections import deque

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


def _dilate(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    """Grow a boolean mask by a disk of `radius_cells`.

    A hand-rolled binary dilation by array shifts: scipy would do this in one
    call, but it is not a dependency of this project and the grid is 50x50.
    """
    grown = mask.copy()
    rows, cols = mask.shape
    for d_row in range(-radius_cells, radius_cells + 1):
        for d_col in range(-radius_cells, radius_cells + 1):
            if d_row * d_row + d_col * d_col > radius_cells * radius_cells:
                continue
            shifted = np.zeros_like(mask)
            row_start, row_stop = max(0, d_row), min(rows, rows + d_row)
            col_start, col_stop = max(0, d_col), min(cols, cols + d_col)
            shifted[row_start:row_stop, col_start:col_stop] = mask[
                row_start - d_row : row_stop - d_row,
                col_start - d_col : col_stop - d_col,
            ]
            grown |= shifted
    return grown


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

        # How far a discovered obstacle must be grown so that a path planned
        # for a point robot still clears the body.
        self._inflate_cells = int(math.ceil(robot_radius_m / cell_size_m))

        # Cells whose centre the robot's own centre could ever occupy: closer
        # to the wall than the body radius simply does not fit.
        limit = half_extent_m - robot_radius_m
        centers = (np.arange(self.n) + 0.5) * cell_size_m - half_extent_m
        in_reach = np.abs(centers) <= limit
        self._center_reachable = np.outer(in_reach, in_reach)

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

    def mark_ray(self, origin, endpoint, hit: bool) -> None:
        """Record one sensor ray: free along its length, maybe occupied at its end.

        Cells are only ever promoted out of UNKNOWN here, so free space seen
        through a doorway cannot later erase an obstacle seen head-on.
        """
        origin_x, origin_y = origin
        end_x, end_y = endpoint
        length = math.hypot(end_x - origin_x, end_y - origin_y)

        # Half-cell steps guarantee no cell along the ray is stepped over.
        steps = max(1, int(length / (self.cell * 0.5)))
        for index in range(steps):  # deliberately excludes the endpoint
            fraction = index / steps
            cell = self.world_to_cell(
                origin_x + fraction * (end_x - origin_x),
                origin_y + fraction * (end_y - origin_y),
            )
            if cell is not None and self.state[cell] == UNKNOWN:
                self.state[cell] = FREE

        if not hit:
            return
        cell = self.world_to_cell(end_x, end_y)
        if cell is None:
            return
        self._hits[cell] += 1
        if self._hits[cell] >= self._hits_to_occupy:
            self.state[cell] = OCCUPIED

    def blocked_mask(self) -> np.ndarray:
        """Cells the robot's centre must not enter.

        That is every confirmed obstacle grown by the body radius, plus the
        band along the walls where the body would not fit. Cells that are
        merely UNKNOWN stay open: in an arena this size, refusing to enter
        unseen space would stop the robot before it had seen anything. The
        reactive layer is what keeps that optimism safe.
        """
        blocked = _dilate(self.state == OCCUPIED, self._inflate_cells)
        blocked |= ~self._center_reachable
        return blocked

    # ---- reporting ----

    def coverage_counts(self) -> tuple[int, int]:
        """(cells covered, cells that could be covered at all)."""
        coverable = self.state != OCCUPIED
        return int((self.covered & coverable).sum()), int(coverable.sum())

    def coverage_fraction(self) -> float:
        """Covered share of the coverable floor, in [0, 1]."""
        covered, coverable = self.coverage_counts()
        return covered / coverable if coverable else 0.0

    # ---- planning ----

    def covers_something_new(self, row: int, col: int) -> bool:
        """Would parking the robot here sweep any ground it has not swept?

        The question is about the footprint, not the single cell: the robot is
        nearly two cells wide, so it can finish a cell by standing beside it.
        """
        for d_row, d_col in self._disk:
            r, c = row + d_row, col + d_col
            if 0 <= r < self.n and 0 <= c < self.n:
                if not self.covered[r, c] and self.state[r, c] != OCCUPIED:
                    return True
        return False

    def nearest_uncovered(self, start):
        """Breadth-first path from `start` to the closest worthwhile cell.

        Because every grid edge costs the same, a plain BFS already yields the
        shortest path; there is nothing for A* to improve on here. Returns None
        once everything still uncovered is walled off or already done.
        """
        blocked = self.blocked_mask()
        if blocked[start]:
            # The robot has ended up somewhere the planner considers illegal --
            # nudged into the wall band, say. Clearing just the start cell is
            # not enough, because every neighbour is usually just as illegal
            # and the search would die immediately; clear enough room around it
            # to escape. This only ever fires when something has already gone
            # wrong, and the reactive layer still guards the drive.
            blocked = blocked.copy()
            row, col = start
            reach = self._inflate_cells + 1
            blocked[
                max(0, row - reach) : row + reach + 1,
                max(0, col - reach) : col + reach + 1,
            ] = False

        came_from = {start: None}
        queue = deque([start])
        while queue:
            current = queue.popleft()
            if current != start and self.covers_something_new(*current):
                return self._trace_path(came_from, current)

            row, col = current
            for neighbour in ((row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)):
                r, c = neighbour
                if 0 <= r < self.n and 0 <= c < self.n:
                    if neighbour not in came_from and not blocked[neighbour]:
                        came_from[neighbour] = current
                        queue.append(neighbour)
        return None

    @staticmethod
    def _trace_path(came_from, cell):
        path = [cell]
        while came_from[path[-1]] is not None:
            path.append(came_from[path[-1]])
        path.reverse()
        return path

    def shortcut(self, path):
        """Drop waypoints the robot can simply drive past.

        BFS returns a staircase of single-cell steps; following it literally
        would make the robot stutter. Greedily jumping to the furthest cell
        still in clear line of sight turns it into a few long straight runs.
        """
        if len(path) < 3:
            return list(path)

        blocked = self.blocked_mask()
        waypoints = [path[0]]
        index = 0
        while index < len(path) - 1:
            furthest = len(path) - 1
            while furthest > index + 1 and not self._line_clear(
                path[index], path[furthest], blocked
            ):
                furthest -= 1
            waypoints.append(path[furthest])
            index = furthest
        return waypoints

    def _line_clear(self, start, end, blocked) -> bool:
        """Is the straight line between two cell centres free of blocked cells?"""
        start_x, start_y = self.cell_center(*start)
        end_x, end_y = self.cell_center(*end)
        length = math.hypot(end_x - start_x, end_y - start_y)
        steps = max(1, int(length / (self.cell * 0.5)))
        for index in range(steps + 1):
            fraction = index / steps
            cell = self.world_to_cell(
                start_x + fraction * (end_x - start_x),
                start_y + fraction * (end_y - start_y),
            )
            if cell is None or blocked[cell]:
                return False
        return True
