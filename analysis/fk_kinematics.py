"""
Forward kinematics for the 5-DOF CNC lathe arm.

Computes the end-effector pose given joint angles, using the exact
transform chain from lathe_arm.urdf.xacro.

Transform chain (URDF joint origins and axes):
  T = Tz(0.025) * Rz(q1) * Tz(L1) * Ry(q2) * Tx(L2) * Ry(q3)
      * Tx(L3) * Ry(q4) * Tx(L4) * Rx(q5) * Tx(L5)

All lengths in metres. Angles in radians.

Usage:
  python3 analysis/fk_kinematics.py
"""

import math
import numpy as np


# Industrial link lengths (metres) ─────────────────────────────────────── #
L1 = 0.180  # base-to-shoulder height
L2 = 0.350  # upper arm
L3 = 0.300  # forearm
L4 = 0.090  # wrist pitch
L5 = 0.065  # wrist roll / ee offset

JOINT_LIMITS = [
    (-1.5708,  1.5708),  # J1 base yaw  ±90°
    (-1.0,     1.9),     # J2 shoulder pitch  -57° to +109°
    (-2.1,     0.2),     # J3 elbow pitch  -120° to +11.5°
    (-1.5708,  1.5708),  # J4 wrist pitch  ±90°
    (-math.pi, math.pi), # J5 wrist roll   ±180°
]


# Elementary transforms ─────────────────────────────────────────────────── #
def Rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0],
                     [0, c,-s, 0],
                     [0, s, c, 0],
                     [0, 0, 0, 1]], dtype=float)

def Ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[ c, 0, s, 0],
                     [ 0, 1, 0, 0],
                     [-s, 0, c, 0],
                     [ 0, 0, 0, 1]], dtype=float)

def Rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,-s, 0, 0],
                     [s, c, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]], dtype=float)

def Tx(d: float) -> np.ndarray:
    return np.array([[1, 0, 0, d],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]], dtype=float)

def Tz(d: float) -> np.ndarray:
    return np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, d],
                     [0, 0, 0, 1]], dtype=float)


# Forward kinematics ────────────────────────────────────────────────────── #
def fk(q: list[float], lengths: dict | None = None) -> np.ndarray:
    """
    Compute 4×4 homogeneous transform from world to end-effector.

    Args:
        q: [q1, q2, q3, q4, q5] in radians
        lengths: optional dict with keys L1..L5 for non-default dimensions

    Returns:
        4×4 numpy array (rotation + translation)
    """
    l1 = (lengths or {}).get("L1", L1)
    l2 = (lengths or {}).get("L2", L2)
    l3 = (lengths or {}).get("L3", L3)
    l4 = (lengths or {}).get("L4", L4)
    l5 = (lengths or {}).get("L5", L5)

    q1, q2, q3, q4, q5 = q

    T = (Tz(0.030) @ Rz(q1)
       @ Tz(l1)    @ Ry(q2)
       @ Tx(l2)    @ Ry(q3)
       @ Tx(l3)    @ Ry(q4)
       @ Tx(l4)    @ Rx(q5)
       @ Tx(l5))
    return T


def ee_position(q: list[float], lengths: dict | None = None) -> np.ndarray:
    """Return (x, y, z) end-effector position for given joint angles."""
    return fk(q, lengths)[:3, 3]


def ee_xyz_rpy(q: list[float], lengths: dict | None = None) -> tuple:
    """Return (x, y, z, roll, pitch, yaw) of end-effector."""
    T = fk(q, lengths)
    x, y, z = T[:3, 3]
    R = T[:3, :3]
    roll  = math.atan2(R[2, 1], R[2, 2])
    pitch = math.atan2(-R[2, 0], math.sqrt(R[2, 1]**2 + R[2, 2]**2))
    yaw   = math.atan2(R[1, 0], R[0, 0])
    return x, y, z, roll, pitch, yaw


# Jacobian (numerical, 5×6) ─────────────────────────────────────────────── #
def jacobian(q: list[float], eps: float = 1e-6) -> np.ndarray:
    """
    Numerical Jacobian: 6×5 matrix mapping joint velocities to
    [vx, vy, vz, wx, wy, wz].

    Uses central finite differences.
    """
    q = list(q)
    J = np.zeros((6, 5))
    for i in range(5):
        qp = q.copy(); qp[i] += eps
        qm = q.copy(); qm[i] -= eps
        Tp = fk(qp); Tm = fk(qm)
        # Linear velocity part
        J[:3, i] = (Tp[:3, 3] - Tm[:3, 3]) / (2 * eps)
        # Angular velocity from skew of (dR/dq * R^T)
        dR = (Tp[:3, :3] - Tm[:3, :3]) / (2 * eps)
        S  = dR @ fk(q)[:3, :3].T
        J[3, i] = S[2, 1]
        J[4, i] = S[0, 2]
        J[5, i] = S[1, 0]
    return J


def manipulability(q: list[float]) -> float:
    """Yoshikawa manipulability index: sqrt(det(J*J^T))."""
    Jl = jacobian(q)[:3]   # linear Jacobian only (3×5)
    return math.sqrt(max(0.0, np.linalg.det(Jl @ Jl.T)))


# Self-test / demo ──────────────────────────────────────────────────────── #
if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)

    print("== Home position (all joints zero) ==")
    T_home = fk([0, 0, 0, 0, 0])
    x, y, z, r, p, yw = ee_xyz_rpy([0, 0, 0, 0, 0])
    print(f"  EE position: x={x:.4f}  y={y:.4f}  z={z:.4f}")
    print(f"  RPY (rad):   r={r:.4f}  p={p:.4f}  yaw={yw:.4f}")
    print()

    test_configs = {
        "J2 = -45°":          [0, -math.pi/4, 0, 0, 0],
        "J2 = -90°, J3 = 90°":[0, -math.pi/2, math.pi/2, 0, 0],
        "J1 = 45° yaw":       [math.pi/4, -math.pi/4, math.pi/4, 0, 0],
    }
    for name, q in test_configs.items():
        pos = ee_position(q)
        m   = manipulability(q)
        print(f"== {name} ==")
        print(f"  EE: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})  "
              f"manipulability={m:.4f}")
    print()
    print("Run analysis/workspace_map.py for the full workspace sweep.")
