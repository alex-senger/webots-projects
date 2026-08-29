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


def test_the_arena_centre_is_stampable_and_the_outer_ring_is_not():
    grid = make_map()
    stampable = grid.stampable_mask()
    assert stampable[25, 25]
    # The robot's centre stops a body radius short of the wall and its
    # footprint reaches one cell further, so the outermost ring stays cold.
    assert not stampable[0, :].any()
    assert not stampable[49, :].any()
    assert not stampable[:, 0].any()
    assert not stampable[:, 49].any()


def test_the_stampable_area_is_the_reachable_centres_grown_by_the_footprint():
    grid = make_map()
    stampable = grid.stampable_mask()
    # Centres reach rows/cols 2..47 (|centre| <= 0.5 - 0.037), and the 3.7 cm
    # footprint adds one cell each way: a 48x48 block spanning indices 1..48.
    assert stampable.sum() == 2304
    assert (~stampable).sum() == 196
    assert stampable[1:49, 1:49].all()


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


def test_a_cell_is_worth_visiting_while_its_footprint_holds_new_ground():
    grid = make_map()
    assert grid.covers_something_new(25, 25)
    grid.stamp_covered(0.0, 0.0)
    assert not grid.covers_something_new(25, 25)


def test_a_cell_whose_footprint_is_all_obstacle_is_not_worth_visiting():
    grid = make_map()
    grid.state[20:30, 20:30] = OCCUPIED
    assert not grid.covers_something_new(25, 25)


def test_the_path_starts_where_the_robot_is_and_ends_somewhere_new():
    grid = make_map()
    grid.stamp_covered(0.0, 0.0)
    path = grid.nearest_uncovered((25, 25))
    assert path is not None
    assert path[0] == (25, 25)
    assert grid.covers_something_new(*path[-1])


def test_the_path_is_connected_step_by_step():
    grid = make_map()
    grid.stamp_covered(0.0, 0.0)
    path = grid.nearest_uncovered((25, 25))
    for (row_a, col_a), (row_b, col_b) in zip(path, path[1:]):
        assert abs(row_a - row_b) + abs(col_a - col_b) == 1


def test_the_nearest_target_is_chosen_not_a_far_one():
    grid = make_map()
    # Cover everything, then reopen one cell a short way off.
    grid.covered[:, :] = True
    grid.covered[25, 31] = False
    path = grid.nearest_uncovered((25, 25))
    assert path is not None
    # Reaching it means getting within a footprint of it, not standing on it:
    # the robot only has to reach column 30 to sweep column 31.
    assert len(path) <= 6


def test_a_fully_covered_map_has_nowhere_left_to_go():
    grid = make_map()
    grid.covered[:, :] = True
    assert grid.nearest_uncovered((25, 25)) is None


def test_the_path_never_crosses_a_blocked_cell():
    grid = make_map()
    grid.covered[:, :] = True
    grid.covered[25, 40] = False
    # A confirmed wall across the arena, with a gap at the very bottom.
    grid.state[5:50, 33] = OCCUPIED
    path = grid.nearest_uncovered((25, 25))
    assert path is not None
    blocked = grid.blocked_mask()
    assert not any(blocked[cell] for cell in path[1:])


def test_an_unreachable_pocket_is_reported_as_nothing_left_to_do():
    grid = make_map()
    grid.covered[:, :] = True
    grid.covered[25, 40] = False
    grid.state[:, 33] = OCCUPIED  # a wall with no gap at all
    assert grid.nearest_uncovered((25, 25)) is None


def test_a_robot_standing_in_a_blocked_cell_can_still_plan_its_way_out():
    grid = make_map()
    grid.stamp_covered(0.0, 0.0)
    # Being pushed against a wall must not strand the planner.
    path = grid.nearest_uncovered((0, 0))
    assert path is not None


def test_shortcut_collapses_a_straight_run_to_its_endpoints():
    grid = make_map()
    path = [(25, col) for col in range(25, 35)]
    assert grid.shortcut(path) == [(25, 25), (25, 34)]


def test_shortcut_keeps_a_corner_it_cannot_cut():
    grid = make_map()
    # An L: east along row 25, then north along column 25.
    path = [(25, col) for col in range(20, 26)] + [(row, 25) for row in range(26, 40)]
    # With nothing in the way the whole L collapses to its two endpoints.
    assert grid.shortcut(path) == [(25, 20), (39, 25)]

    # Put a barrier across the diagonal the shortcut just took.
    grid.state[30:35, 21] = OCCUPIED
    shortened = grid.shortcut(path)
    assert shortened[0] == path[0]
    assert shortened[-1] == path[-1]
    assert len(shortened) > 2  # the corner had to survive
    assert len(shortened) < len(path)


def test_shortcut_leaves_a_trivial_path_alone():
    grid = make_map()
    assert grid.shortcut([(25, 25)]) == [(25, 25)]
