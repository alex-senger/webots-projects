import math

import pytest

from epucklib import epuck
from epucklib.odometry import DeadReckoning, Pose, integrate_distance

R = epuck.WHEEL_RADIUS_M
L = epuck.AXLE_LENGTH_M


def test_a_fresh_pose_is_the_origin():
    pose = Pose()
    assert (pose.x, pose.y, pose.theta) == (0.0, 0.0, 0.0)


def test_equal_wheel_rotation_drives_straight_along_x():
    odom = DeadReckoning(0.0, 0.0)
    turn = 1.0 / R  # one metre's worth of wheel rotation
    pose = odom.update(turn, turn)
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(0.0)
    assert pose.theta == pytest.approx(0.0)


def test_opposite_wheel_rotation_turns_in_place():
    odom = DeadReckoning(0.0, 0.0)
    # Right wheel forward, left wheel back by the same amount: pure rotation.
    quarter = (math.pi / 2) * (L / 2) / R
    pose = odom.update(-quarter, quarter)
    assert pose.x == pytest.approx(0.0)
    assert pose.y == pytest.approx(0.0)
    assert pose.theta == pytest.approx(math.pi / 2)


def test_many_small_steps_trace_a_circle_back_to_the_start():
    odom = DeadReckoning(0.0, 0.0)
    # A full turn on the spot, in 360 increments, must return to theta ~ 0.
    step = (2 * math.pi / 360) * (L / 2) / R
    left = right = 0.0
    for _ in range(360):
        left -= step
        right += step
        odom.update(left, right)
    assert odom.pose.theta == pytest.approx(0.0, abs=1e-6)
    assert math.hypot(odom.pose.x, odom.pose.y) == pytest.approx(0.0, abs=1e-9)


def test_step_ds_reports_the_distance_of_the_last_update_only():
    odom = DeadReckoning(0.0, 0.0)
    turn = 0.5 / R
    odom.update(turn, turn)
    assert odom.step_ds == pytest.approx(0.5)
    odom.update(2 * turn, 2 * turn)
    assert odom.step_ds == pytest.approx(0.5)


def test_a_starting_pose_can_be_supplied():
    odom = DeadReckoning(0.0, 0.0, Pose(1.0, 2.0, math.pi / 2))
    turn = 0.1 / R
    pose = odom.update(turn, turn)
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(2.1)


def test_integrate_distance_ignores_direction():
    forward = integrate_distance(1.0, 1.0, 1.0)
    backward = integrate_distance(-1.0, -1.0, 1.0)
    assert forward == pytest.approx(backward)
    assert forward == pytest.approx(R)
    # Spinning on the spot covers no ground.
    assert integrate_distance(-1.0, 1.0, 1.0) == pytest.approx(0.0)
