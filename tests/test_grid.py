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


def test_a_ray_that_hits_nothing_only_clears_free_space():
    grid = make_map()
    grid.mark_ray((0.0, 0.0), (0.0, 0.1), hit=False)
    assert (grid.state == OCCUPIED).sum() == 0
    assert (grid.state == FREE).sum() > 0
    # The cells along the ray, but not beyond its end.
    assert grid.state[25, 25] == FREE
    assert grid.state[29, 25] == FREE
    assert grid.state[40, 25] == UNKNOWN


def test_one_sighting_is_not_enough_to_believe_an_obstacle():
    grid = make_map()
    grid.mark_ray((0.0, 0.0), (0.0, 0.1), hit=True)
    assert (grid.state == OCCUPIED).sum() == 0


def test_a_second_sighting_confirms_the_obstacle():
    grid = make_map()
    for _ in range(2):
        grid.mark_ray((0.0, 0.0), (0.0, 0.1), hit=True)
    assert grid.state[grid.world_to_cell(0.0, 0.1)] == OCCUPIED


def test_a_confirmed_obstacle_survives_later_free_rays():
    grid = make_map()
    for _ in range(2):
        grid.mark_ray((0.0, 0.0), (0.0, 0.1), hit=True)
    obstacle = grid.world_to_cell(0.0, 0.1)
    grid.mark_ray((0.0, 0.0), (0.0, 0.3), hit=False)
    assert grid.state[obstacle] == OCCUPIED


def test_a_ray_leaving_the_arena_is_ignored_rather_than_wrapping():
    grid = make_map()
    grid.mark_ray((0.49, 0.0), (0.60, 0.0), hit=True)
    assert (grid.state == OCCUPIED).sum() == 0


def test_dilate_grows_a_single_cell_into_a_disk():
    from epucklib.grid import _dilate

    mask = np.zeros((11, 11), dtype=bool)
    mask[5, 5] = True
    grown = _dilate(mask, 2)
    assert grown[5, 5]
    assert grown[3, 5] and grown[7, 5] and grown[5, 3] and grown[5, 7]
    assert grown[4, 4]  # diagonal neighbour, within radius 2
    assert not grown[3, 3]  # 2.83 cells away, outside radius 2
    assert not grown[8, 5]


def test_dilate_near_an_edge_does_not_wrap_around():
    from epucklib.grid import _dilate

    mask = np.zeros((11, 11), dtype=bool)
    mask[0, 0] = True
    grown = _dilate(mask, 2)
    assert grown[0, 0] and grown[2, 0] and grown[0, 2]
    assert not grown[10, 10]
    assert not grown[10, 0]


def test_the_border_is_blocked_because_the_body_will_not_fit():
    grid = make_map()
    blocked = grid.blocked_mask()
    # The robot centre cannot come closer than 3.7 cm to the wall at 0.5 m.
    assert blocked[0, 0]
    assert blocked[25, 0]
    assert not blocked[25, 25]


def test_a_discovered_obstacle_is_inflated_by_the_body_radius():
    grid = make_map()
    for _ in range(2):
        grid.mark_ray((0.0, 0.0), (0.0, 0.1), hit=True)
    obstacle_row, obstacle_col = grid.world_to_cell(0.0, 0.1)
    blocked = grid.blocked_mask()
    assert blocked[obstacle_row, obstacle_col]
    # 3.7 cm of body over 2 cm cells inflates by two cells.
    assert blocked[obstacle_row, obstacle_col + 2]
    assert not blocked[obstacle_row, obstacle_col + 4]


def test_the_blocked_mask_is_a_fresh_array_each_time():
    grid = make_map()
    first = grid.blocked_mask()
    first[25, 25] = True
    assert not grid.blocked_mask()[25, 25]
