import pytest

pytest.importorskip("rclpy")                      # pot_teleop imports ROS

from lathe_arm_teach.pot_teleop import SPEED, move_time


def test_no_move_gives_the_minimum():
    assert move_time([0, 0, 0, 0], [0, 0, 0, 0], 0.5) == 0.5


def test_slowest_joint_sets_the_time():
    t = move_time([0, 0, 0, 0], [1.0, 0.1, 0.1, 0.1], 0.0)
    assert t == pytest.approx(1.0 / SPEED[0])


def test_boom_is_the_slow_one_for_equal_moves():
    t = move_time([0, 0, 0, 0], [0.5, 0.5, 0.5, 0.5], 0.0)
    assert t == pytest.approx(0.5 / SPEED[1])


def test_direction_does_not_matter():
    assert move_time([0, 0, 0, 0], [0.4, 0, 0, 0], 0) == move_time([0.4, 0, 0, 0], [0, 0, 0, 0], 0)


def test_speeds_stay_under_urdf_limits():
    urdf = [0.6, 0.4, 2.0, 2.0]
    assert all(s <= u for s, u in zip(SPEED, urdf))
