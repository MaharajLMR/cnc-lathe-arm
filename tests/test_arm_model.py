import math
import random

import pytest

from lathe_arm_teach import arm_model as am

HOME = [0.0, math.radians(10), math.radians(-10), 0.0]


def test_dimensions_from_the_photo_and_notes():
    assert am.PIVOT_Z == 1.50
    assert am.BOOM == 1.00


def test_actuator_stroke():
    assert am.ACT_MIN == pytest.approx(0.767, abs=1e-3)
    assert am.ACT_MAX == pytest.approx(1.154, abs=1e-3)


def test_actuator_length_angle_round_trip():
    lo, hi = am.LIMITS["j2_boom_lift"]
    for i in range(21):
        a = lo + (hi - lo) * i / 20
        assert am.boom_angle(am.actuator_length(a)) == pytest.approx(a, abs=1e-9)


def test_actuator_length_grows_with_boom_angle():
    assert am.actuator_length(0.0) < am.actuator_length(0.5)


def test_boom_angle_rejects_impossible_length():
    with pytest.raises(ValueError):
        am.boom_angle(3.0)


def test_actuator_pose_matches_length():
    pitch, length = am.actuator_pose(0.3)
    assert length == pytest.approx(am.actuator_length(0.3))
    assert 0.0 < pitch < math.pi


def test_level_wrist_keeps_gripper_vertical():
    for q2 in (-0.4, 0.0, 0.3, 0.7):
        q = [0.0, q2, am.level_wrist(q2), 0.0]
        x, _, z = am.tool_position(q)
        # straight down means the fingertips are TOOL below the boom tip
        wx, _, wz = am.wrist_position(q)
        assert x == pytest.approx(wx, abs=1e-9)
        assert wz - z == pytest.approx(am.TOOL)


def test_home_pose():
    x, y, z = am.tool_position(HOME)
    assert y == pytest.approx(0.0)
    assert z < am.PIVOT_Z


def test_base_yaw_rotates_about_the_column():
    a = am.tool_position([0.0, 0.1, -0.1, 0.0])
    b = am.tool_position([1.0, 0.1, -0.1, 0.0])
    assert math.hypot(a[0], a[1]) == pytest.approx(math.hypot(b[0], b[1]))
    assert a[2] == pytest.approx(b[2])


def test_limits():
    assert am.within_limits([0.0, 0.0, 0.0, 0.0, 0.02])
    assert not am.within_limits([0.0, math.radians(60), 0.0, 0.0, 0.02])
    assert not am.within_limits([0.0, 0.0, 0.0, 0.0, 0.05])


def test_inverse_round_trip():
    random.seed(1)
    n = 0
    for _ in range(200):
        q = [random.uniform(-2, 2), random.uniform(-0.5, 0.78), 0.0, 0.0]
        q[2] = am.level_wrist(q[1]) + random.uniform(-0.3, 0.3)
        if not am.within_limits(q + [0.0]):
            continue
        p = am.tool_position(q)
        sol = am.inverse(*p, pitch_hint=q[1] + q[2])
        assert sol is not None
        assert am.tool_position(sol) == pytest.approx(p, abs=1e-9)
        n += 1
    assert n > 100


def test_inverse_out_of_reach():
    assert am.inverse(3.0, 0.0, 1.0) is None


def test_jacobian_matches_finite_difference():
    q = [0.4, 0.2, -0.1, 0.0]
    jac = am.jacobian(q)
    for i in range(3):
        dq = list(q)
        dq[i] += 1e-4
        p0, p1 = am.tool_position(q), am.tool_position(dq)
        for r in range(3):
            assert jac[r][i] == pytest.approx((p1[r] - p0[r]) / 1e-4, abs=1e-3)
