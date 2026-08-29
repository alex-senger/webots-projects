import pytest

from epucklib.sweep import lane_waypoints


def test_two_waypoints_per_lane():
    points = lane_waypoints(limit_m=0.463, spacing_m=0.06)
    assert len(points) % 2 == 0
    assert len(points) >= 2


def test_the_sweep_starts_and_ends_on_the_outermost_lanes():
    points = lane_waypoints(limit_m=0.463, spacing_m=0.06)
    assert points[0][0] == pytest.approx(-0.463)
    assert points[-1][0] == pytest.approx(0.463)


def test_lanes_are_never_further_apart_than_requested():
    points = lane_waypoints(limit_m=0.463, spacing_m=0.06)
    xs = sorted({round(x, 9) for x, _ in points})
    gaps = [b - a for a, b in zip(xs, xs[1:])]
    assert all(gap <= 0.06 + 1e-9 for gap in gaps)
    # Evenly spaced, so the sweep looks deliberate rather than lopsided.
    assert max(gaps) - min(gaps) < 1e-9


def test_the_arena_needs_seventeen_lanes_at_six_centimetre_spacing():
    points = lane_waypoints(limit_m=0.463, spacing_m=0.06)
    assert len({round(x, 9) for x, _ in points}) == 17


def test_the_path_is_serpentine_so_no_lane_is_driven_twice():
    points = lane_waypoints(limit_m=0.463, spacing_m=0.06)
    # Each lane runs the opposite way to the one before it.
    for index in range(0, len(points) - 2, 2):
        this_lane = points[index + 1][1] - points[index][1]
        next_lane = points[index + 3][1] - points[index + 2][1]
        assert this_lane * next_lane < 0


def test_every_waypoint_stays_inside_the_limit():
    points = lane_waypoints(limit_m=0.463, spacing_m=0.06)
    assert all(abs(x) <= 0.463 + 1e-9 and abs(y) <= 0.463 + 1e-9 for x, y in points)


def test_a_spacing_wider_than_the_arena_still_gives_both_edges():
    points = lane_waypoints(limit_m=0.463, spacing_m=5.0)
    assert len({round(x, 9) for x, _ in points}) == 2
    assert points[0][0] == pytest.approx(-0.463)
    assert points[-1][0] == pytest.approx(0.463)
