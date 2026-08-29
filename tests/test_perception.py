import math

import pytest

from epucklib import epuck, perception
from epucklib.odometry import Pose


def test_contact_reading_maps_to_zero_distance():
    assert perception.raw_to_distance(4095.0) == pytest.approx(0.0)
    assert perception.raw_to_distance(9999.0) == pytest.approx(0.0)


def test_weak_readings_mean_nothing_in_range():
    assert perception.raw_to_distance(67.19) == pytest.approx(epuck.IR_MAX_RANGE_M)
    assert perception.raw_to_distance(10.0) == pytest.approx(epuck.IR_MAX_RANGE_M)
    assert perception.raw_to_distance(0.0) == pytest.approx(epuck.IR_MAX_RANGE_M)


def test_every_table_entry_inverts_to_its_own_distance():
    for distance, raw in zip(epuck.LOOKUP_DISTANCE_M, epuck.LOOKUP_RAW):
        assert perception.raw_to_distance(raw) == pytest.approx(distance, abs=1e-9)


def test_distance_falls_as_the_reading_rises():
    readings = [80.0, 150.0, 400.0, 1500.0, 3000.0]
    distances = [perception.raw_to_distance(r) for r in readings]
    assert all(a > b for a, b in zip(distances, distances[1:]))


def test_read_distances_converts_the_whole_ring():
    distances = perception.read_distances([4095.0] * 8)
    assert len(distances) == 8
    assert all(d == pytest.approx(0.0) for d in distances)


def test_ray_from_the_origin_leaves_through_the_right_hand_sensor():
    # ps2 sits at (0, -0.031) looking 90 degrees to the right.
    origin, endpoint = perception.sensor_ray(Pose(0.0, 0.0, 0.0), 2, 0.05)
    assert origin == pytest.approx((0.0, -0.031))
    assert endpoint[0] == pytest.approx(0.0, abs=1e-3)
    assert endpoint[1] == pytest.approx(-0.081, abs=1e-3)


def test_ray_rotates_and_translates_with_the_robot():
    # Facing +y (theta = pi/2), the robot's right-hand side points to +x.
    origin, endpoint = perception.sensor_ray(Pose(1.0, 2.0, math.pi / 2), 2, 0.05)
    assert origin == pytest.approx((1.031, 2.0), abs=1e-9)
    assert endpoint[0] == pytest.approx(1.081, abs=1e-3)
    assert endpoint[1] == pytest.approx(2.0, abs=1e-3)


def test_nothing_in_range_produces_no_repulsion():
    fx, fy, front = perception.repulsion([epuck.IR_MAX_RANGE_M] * 8)
    assert (fx, fy, front) == pytest.approx((0.0, 0.0, 0.0))


def test_an_obstacle_on_the_right_pushes_left():
    distances = [epuck.IR_MAX_RANGE_M] * 8
    distances[2] = 0.01  # ps2, straight out to the right
    fx, fy, _ = perception.repulsion(distances)
    assert fy > 0.0  # pushed toward +y, i.e. left
    # ps2 is mounted a hair off exactly 90 degrees, so fx is near zero, not zero.
    assert abs(fx) < 0.01


def test_a_closer_obstacle_pushes_harder():
    near = list(perception.repulsion([0.01 if i == 2 else epuck.IR_MAX_RANGE_M for i in range(8)]))
    far = list(perception.repulsion([0.05 if i == 2 else epuck.IR_MAX_RANGE_M for i in range(8)]))
    assert near[1] > far[1] > 0.0


def test_a_head_on_obstacle_pushes_backward_and_raises_the_front_factor():
    distances = [epuck.IR_MAX_RANGE_M] * 8
    distances[0] = distances[7] = 0.01
    fx, _, front = perception.repulsion(distances)
    assert fx < 0.0
    assert front > 0.8


def test_only_the_forward_sensors_contribute_to_the_front_factor():
    distances = [epuck.IR_MAX_RANGE_M] * 8
    distances[3] = distances[4] = 0.005  # rear-facing pair, nearly touching
    _, _, front = perception.repulsion(distances)
    assert front == pytest.approx(0.0)
