import numpy as np
import pytest

from epucklib import epuck
from epucklib.grid import FREE, OCCUPIED, UNKNOWN, CoverageMap


def make_map():
    return CoverageMap(half_extent_m=0.5, cell_size_m=0.02, robot_radius_m=0.037)


def test_the_arena_divides_into_fifty_by_fifty_cells():
    grid = make_map()
    assert grid.n == 50
    assert grid.covered.shape == (50, 50)
    assert grid.state.shape == (50, 50)
    assert not grid.covered.any()
    assert (grid.state == UNKNOWN).all()


def test_the_centre_of_the_arena_is_the_middle_of_the_grid():
    grid = make_map()
    assert grid.world_to_cell(0.0, 0.0) == (25, 25)
    # Row follows +y, column follows +x.
    assert grid.world_to_cell(0.1, 0.0) == (25, 30)
    assert grid.world_to_cell(0.0, 0.1) == (30, 25)


def test_the_corners_land_on_the_extreme_cells():
    grid = make_map()
    assert grid.world_to_cell(-0.499, -0.499) == (0, 0)
    assert grid.world_to_cell(0.499, 0.499) == (49, 49)


def test_points_outside_the_arena_have_no_cell():
    grid = make_map()
    assert grid.world_to_cell(0.6, 0.0) is None
    assert grid.world_to_cell(0.0, -0.51) is None


def test_cell_centre_round_trips_back_to_the_same_cell():
    grid = make_map()
    for cell in ((0, 0), (25, 25), (49, 13), (7, 49)):
        x, y = grid.cell_center(*cell)
        assert grid.world_to_cell(x, y) == cell


def test_standing_still_covers_a_disk_of_nine_cells():
    grid = make_map()
    grid.stamp_covered(0.0, 0.0)
    # A 3.7 cm radius over 2 cm cells reaches the diagonal neighbours
    # (2.83 cm) but not the cell two steps away (4 cm).
    assert grid.covered.sum() == 9
    assert grid.covered[24:27, 24:27].all()


def test_the_stamped_disk_is_also_recorded_as_free_space():
    grid = make_map()
    grid.stamp_covered(0.0, 0.0)
    assert (grid.state[24:27, 24:27] == FREE).all()
    assert grid.state[0, 0] == UNKNOWN


def test_stamping_never_overwrites_a_known_obstacle():
    grid = make_map()
    grid.state[25, 25] = OCCUPIED
    grid.stamp_covered(0.0, 0.0)
    assert grid.state[25, 25] == OCCUPIED


def test_stamping_near_the_wall_is_clipped_not_wrapped():
    grid = make_map()
    grid.stamp_covered(-0.49, -0.49)
    assert grid.covered[0, 0]
    # Nothing may leak to the opposite side of the arena.
    assert not grid.covered[49, 49]


def test_coverage_is_measured_against_the_cells_that_can_be_covered():
    grid = make_map()
    assert grid.coverage_counts() == (0, 2500)
    assert grid.coverage_fraction() == pytest.approx(0.0)

    grid.state[grid.state == UNKNOWN] = FREE
    grid.state[0, :] = OCCUPIED  # a wall of 50 cells nobody can reach
    covered, coverable = grid.coverage_counts()
    assert coverable == 2450

    grid.covered[:, :] = True  # includes the obstacle row
    covered, coverable = grid.coverage_counts()
    assert covered == 2450  # obstacle cells never count as covered
    assert grid.coverage_fraction() == pytest.approx(1.0)


def test_a_fully_blocked_map_reports_zero_rather_than_dividing_by_zero():
    grid = make_map()
    grid.state[:, :] = OCCUPIED
    assert grid.coverage_fraction() == 0.0
