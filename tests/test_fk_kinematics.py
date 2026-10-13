"""
Unit tests for the pure-Python forward kinematics module.

Run with:  python3 -m pytest tests/test_fk_kinematics.py -v
"""

import sys
import os
import math
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "analysis"))
from fk_kinematics import (
    fk, ee_position, ee_xyz_rpy, jacobian, manipulability,
    Rx, Ry, Rz, Tx, Tz,
    L1, L2, L3, L4, L5, JOINT_LIMITS,
)


# ── Elementary transform sanity checks ───────────────────────────────────── #

def test_Rz_90_rotates_x_to_y():
    T = Rz(math.pi / 2)
    v = T @ np.array([1, 0, 0, 1])
    assert abs(v[0]) < 1e-9
    assert abs(v[1] - 1.0) < 1e-9


def test_Ry_90_rotates_z_to_x():
    T = Ry(math.pi / 2)
    v = T @ np.array([0, 0, 1, 1])
    assert abs(v[0] - 1.0) < 1e-9
    assert abs(v[2]) < 1e-9


def test_Tx_translates():
    T = Tx(0.5)
    v = T @ np.array([0, 0, 0, 1])
    assert abs(v[0] - 0.5) < 1e-9


# ── FK output shape and type ──────────────────────────────────────────────── #

def test_fk_returns_4x4():
    T = fk([0, 0, 0, 0, 0])
    assert T.shape == (4, 4)


def test_fk_rotation_is_orthogonal():
    """Rotation sub-matrix must be orthogonal (det = +1)."""
    T = fk([0.3, -0.5, 0.8, -0.3, 1.0])
    R = T[:3, :3]
    det = np.linalg.det(R)
    assert abs(det - 1.0) < 1e-9, f"det(R)={det:.6f}, expected 1.0"
    # R^T * R = I
    err = np.max(np.abs(R.T @ R - np.eye(3)))
    assert err < 1e-9, f"R^T R != I, max err={err:.2e}"


def test_fk_home_position():
    """At all-zero joints, EE should be at (L2+L3+L4+L5, 0, 0.030+L1)."""
    x, y, z, r, p, yw = ee_xyz_rpy([0, 0, 0, 0, 0])
    expected_x = L2 + L3 + L4 + L5
    expected_z = 0.030 + L1
    assert abs(x - expected_x) < 1e-6, f"x={x:.4f}, expected {expected_x:.4f}"
    assert abs(y) < 1e-9
    assert abs(z - expected_z) < 1e-6, f"z={z:.4f}, expected {expected_z:.4f}"


def test_j1_yaw_rotates_xy_plane():
    """J1 (yaw) should rotate the EE position about the Z axis."""
    pos0 = ee_position([0, 0, 0, 0, 0])
    pos90 = ee_position([math.pi / 2, 0, 0, 0, 0])
    # After 90° yaw: x_new ≈ -y_old, y_new ≈ x_old, z unchanged
    assert abs(pos90[1] - pos0[0]) < 1e-6, "y_new should equal x_old after 90° yaw"
    assert abs(pos90[0]) < 1e-6, "x_new should be ~0 after 90° yaw"
    assert abs(pos90[2] - pos0[2]) < 1e-6, "Z should be unchanged by yaw"


def test_j2_shoulder_pitch_raises_ee():
    """Pitching shoulder up (negative J2) should raise the EE z-height."""
    pos0 = ee_position([0,  0, 0, 0, 0])
    posup = ee_position([0, -math.pi / 4, 0, 0, 0])
    assert posup[2] > pos0[2], "EE should rise when shoulder pitches up"


def test_j5_roll_does_not_change_position():
    """Wrist roll (J5) changes orientation only, not EE position."""
    q_base = [0.3, -0.4, 0.5, -0.2, 0.0]
    pos0 = ee_position(q_base)
    q_rolled = q_base.copy()
    q_rolled[4] = math.pi / 3
    pos_r = ee_position(q_rolled)
    diff = np.linalg.norm(pos_r - pos0)
    assert diff < 1e-6, f"Wrist roll should not move EE, got diff={diff:.2e}"


# ── Jacobian and manipulability ───────────────────────────────────────────── #

def test_jacobian_shape():
    J = jacobian([0.1, -0.3, 0.5, -0.1, 0.2])
    assert J.shape == (6, 5)


def test_manipulability_positive():
    """Manipulability must be non-negative."""
    for _ in range(10):
        q = [math.uniform(lo, hi) if False else
             (lo + hi) / 2 + (hi - lo) / 4 * math.sin(_ * 1.7)
             for lo, hi in JOINT_LIMITS]
        m = manipulability(q)
        assert m >= 0.0


def test_home_singularity():
    """Home pose (all-zero) is singular for this arm geometry — expected."""
    m = manipulability([0, 0, 0, 0, 0])
    assert m < 1e-6, (
        f"Home pose should be singular (arm fully extended in X), got m={m:.2e}"
    )


def test_non_trivial_config_not_singular():
    """A generic off-axis config should have positive manipulability."""
    q = [0.5, -0.7, 1.2, -0.4, 0.3]
    m = manipulability(q)
    assert m > 1e-4, f"Expected positive manipulability, got {m:.4f}"


# ── Joint limits coverage ────────────────────────────────────────────────── #

def test_fk_at_joint_limits():
    """FK should not crash at the boundary of any joint limit."""
    for i, (lo, hi) in enumerate(JOINT_LIMITS):
        for val in (lo, hi, (lo + hi) / 2):
            q = [0.0] * 5
            q[i] = val
            pos = ee_position(q)
            assert np.isfinite(pos).all(), f"Non-finite EE at joint {i+1}={val:.3f}"
