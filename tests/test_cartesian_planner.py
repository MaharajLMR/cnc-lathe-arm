"""Tests for analysis/cartesian_path_planner.py, no ROS required."""

import sys
import math
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "analysis"))
from cartesian_path_planner import CartesianPathPlanner, linearity_error_mm, jacobian_ik
from fk_kinematics import ee_position, JOINT_LIMITS

Q_MIN = np.array([lo for lo, _ in JOINT_LIMITS])
Q_MAX = np.array([hi for _, hi in JOINT_LIMITS])


@pytest.fixture(scope="module")
def planner():
    return CartesianPathPlanner()


@pytest.fixture(scope="module")
def cycle_segments(planner):
    return planner.plan_load_cycle()


def test_linear_segment_is_straight(planner):
    p0 = np.array([0.4, 0.0, 0.15])
    p1 = np.array([0.4, 0.0, 0.05])
    pts = planner.linear_segment(p0, p1, n_points=50)
    err = linearity_error_mm(pts)
    assert err < 0.001, f"Linearity error {err:.4f} mm, linear_segment must be a straight line"


def test_quintic_zero_boundary_velocity(planner):
    """Quintic path must have near-zero velocity at start and end."""
    p0 = np.array([0.30, 0.0, 0.20])
    p1 = np.array([0.55, 0.0, 0.15])
    times, ee_pts = planner.quintic_cartesian(p0, p1, T=1.5, dt=0.005)
    # velocity ≈ finite difference
    v_start = np.linalg.norm(ee_pts[1] - ee_pts[0]) / 0.005
    v_end   = np.linalg.norm(ee_pts[-1] - ee_pts[-2]) / 0.005
    assert v_start < 0.05, f"Start velocity {v_start:.4f} m/s is not near zero"
    assert v_end   < 0.05, f"End velocity {v_end:.4f} m/s is not near zero"


def test_no_joint_limit_violations_in_load_cycle(planner, cycle_segments):
    all_jc = np.vstack([s["joint_configs"] for s in cycle_segments])
    violations = planner.check_joint_limit_violations(all_jc)
    assert violations == [], (
        f"{len(violations)} joint limit violations found: {violations[:3]}"
    )


def test_linear_segments_are_named_correctly(cycle_segments):
    linear_names = {s["name"] for s in cycle_segments if s["path_type"] == "LINEAR"}
    assert "approach_to_insert" in linear_names, "Insert approach must be LINEAR"
    assert "insert_to_retract" in linear_names, "Retract must be LINEAR"
    assert "approach_to_grasp" in linear_names, "Grasp approach must be LINEAR"


def test_linear_segments_have_low_linearity_error(cycle_segments):
    for s in cycle_segments:
        if s["path_type"] == "LINEAR":
            err = linearity_error_mm(s["ee_positions"])
            assert err < 0.1, (
                f"Segment '{s['name']}' linearity error {err:.4f} mm > 0.1 mm"
            )


def test_cycle_has_seven_segments(cycle_segments):
    assert len(cycle_segments) == 7


def test_jacobian_ik_converges_to_target():
    target = np.array([0.35, -0.10, 0.12])
    q_seed = np.array([0.0, 0.8, -1.5, 0.5, 0.0])
    q_sol = jacobian_ik(target, q_seed)
    reached = ee_position(q_sol)
    err = np.linalg.norm(target - reached)
    assert err < 5e-3, f"IK did not converge: error {err*1000:.2f} mm"


def test_joint_configs_shape(cycle_segments):
    for s in cycle_segments:
        jc = s["joint_configs"]
        assert jc.ndim == 2, f"joint_configs must be 2-D"
        assert jc.shape[1] == 5, f"Must have 5 joints, got {jc.shape[1]}"


def test_ee_positions_shape(cycle_segments):
    for s in cycle_segments:
        ee = s["ee_positions"]
        assert ee.ndim == 2, f"ee_positions must be 2-D"
        assert ee.shape[1] == 3, f"Must have 3 Cartesian coords, got {ee.shape[1]}"
