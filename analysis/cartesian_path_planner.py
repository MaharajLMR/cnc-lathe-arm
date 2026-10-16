"""
Cartesian path planner for the 5-DOF CNC lathe loading arm.

For CNC lathe loading the end-effector MUST approach the chuck along a straight
Cartesian line, joint-interpolated arcs can clip the workpart or machine frame.

Segments:
  free-motion  → quintic joint-space interpolation
  linear       → straight-line Cartesian + numerical IK at each sample point

Usage:
  python3 analysis/cartesian_path_planner.py
"""

import sys
import json
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent))
from fk_kinematics import fk, ee_position, jacobian, JOINT_LIMITS, L1, L2, L3, L4, L5

# Joint limits as arrays ────────────────────────────────────────────────── #
Q_MIN = np.array([lo for lo, _ in JOINT_LIMITS])
Q_MAX = np.array([hi for _, hi in JOINT_LIMITS])


# Numerical IK via damped least-squares ────────────────────────────────── #
def jacobian_ik(target_pos, q_init, max_iter=300, tol=1e-5, lam=0.01):
    """
    Numerical IK using damped least-squares (DLS) with the position Jacobian.

    q_{k+1} = q_k + J^T (JJ^T + λI)^{-1} err
    """
    q = np.array(q_init, dtype=float)
    for _ in range(max_iter):
        pos = ee_position(q)
        err = target_pos - pos
        if np.linalg.norm(err) < tol:
            break
        J = jacobian(q)[:3, :]          # 3×5 position rows
        JJT = J @ J.T
        delta_q = J.T @ np.linalg.solve(JJT + lam * np.eye(3), err)
        q = np.clip(q + 0.5 * delta_q, Q_MIN, Q_MAX)
    return q


# Quintic polynomial (minimum-jerk) scalar helper ──────────────────────── #
def _quintic_coeff(q0, qf, T):
    """Return [a0..a5] for min-jerk quintic with zero boundary velocities/accels."""
    a0, a1, a2 = q0, 0.0, 0.0
    T3 = T ** 3
    T4 = T ** 4
    T5 = T ** 5
    dq = qf - q0
    a3 = 10 * dq / T3
    a4 = -15 * dq / T4
    a5 = 6 * dq / T5
    return np.array([a0, a1, a2, a3, a4, a5])


def _eval_quintic(c, t):
    """Evaluate quintic polynomial and its 1st derivative at t."""
    t2, t3, t4, t5 = t**2, t**3, t**4, t**5
    q   = c[0] + c[1]*t + c[2]*t2 + c[3]*t3 + c[4]*t4 + c[5]*t5
    dq  = c[1] + 2*c[2]*t + 3*c[3]*t2 + 4*c[4]*t3 + 5*c[5]*t4
    ddq = 2*c[2] + 6*c[3]*t + 12*c[4]*t2 + 20*c[5]*t3
    return q, dq, ddq


# CartesianPathPlanner ──────────────────────────────────────────────────── #
class CartesianPathPlanner:
    def __init__(self):
        self.joint_limits = JOINT_LIMITS

    # primitives ────────────────────────────────────────────────────────── #
    def linear_segment(self, p_start, p_end, n_points=50):
        """Linear interpolation in Cartesian space → n_points×3 array."""
        t = np.linspace(0.0, 1.0, n_points)
        return np.outer(1 - t, p_start) + np.outer(t, p_end)

    def quintic_cartesian(self, p_start, p_end, T, dt=0.02):
        """
        Minimum-jerk profile along straight Cartesian path.
        Returns: (times, ee_positions [N×3], joint_configs [N×5])
        """
        times = np.arange(0.0, T + dt, dt)
        ee_pts = np.zeros((len(times), 3))
        coeffs = [_quintic_coeff(p_start[i], p_end[i], T) for i in range(3)]
        for k, t in enumerate(times):
            for i in range(3):
                ee_pts[k, i], _, _ = _eval_quintic(coeffs[i], min(t, T))
        return times, ee_pts

    def joint_quintic(self, q_start, q_end, n_points=30):
        """Quintic interpolation in joint space → n_points×5 array."""
        T = 1.0
        configs = np.zeros((n_points, 5))
        ts = np.linspace(0.0, T, n_points)
        for j in range(5):
            c = _quintic_coeff(q_start[j], q_end[j], T)
            for k, t in enumerate(ts):
                configs[k, j], _, _ = _eval_quintic(c, t)
        return configs

    # IK-backed Cartesian path ──────────────────────────────────────────── #
    def cartesian_to_joints(self, ee_pts, q_seed):
        """
        Solve IK for each Cartesian point, warm-starting from previous solution.
        Returns: N×5 joint configs.
        """
        q = np.array(q_seed, dtype=float)
        configs = np.zeros((len(ee_pts), 5))
        for k, p in enumerate(ee_pts):
            q = jacobian_ik(p, q)
            configs[k] = q
        return configs

    # Full load cycle ───────────────────────────────────────────────────── #
    def plan_load_cycle(self):
        """
        Plan a complete load cycle. Approach / retract / insert segments use
        straight Cartesian paths; free-motion segments use joint-space quintics.
        """
        # Waypoint joint configs from example_load_cycle.yaml ──────────── #
        q_home            = np.array([0.00,  0.00, -0.00,  0.00, 0.0])
        q_approach_part   = np.array([0.00,  0.80, -1.50,  0.50, 0.0])
        q_grasp_part      = np.array([0.00,  0.90, -1.70,  0.60, 0.0])
        q_lift            = np.array([0.00,  0.50, -1.20,  0.40, 0.0])
        q_approach_chuck  = np.array([1.20,  0.60, -1.40,  0.40, 0.0])
        q_insert          = np.array([1.20,  0.65, -1.50,  0.45, 0.0])
        q_retract         = np.array([1.20,  0.50, -1.30,  0.35, 0.0])

        def p(q): return ee_position(q)

        segments = []

        def joint_seg_with_ee(q_s, q_e, n=30):
            jc = self.joint_quintic(q_s, q_e, n_points=n)
            ee = np.array([ee_position(q) for q in jc])
            return jc, ee

        # 1. home → approach_part  [joint space]
        jc, ee_pts = joint_seg_with_ee(q_home, q_approach_part)
        segments.append({
            "name": "home_to_approach",
            "path_type": "JOINT",
            "ee_positions": ee_pts,
            "joint_configs": jc,
        })

        # 2. approach_part → grasp_part  [LINEAR, straight down to part]
        ee_pts = self.linear_segment(p(q_approach_part), p(q_grasp_part), n_points=20)
        jc = self.cartesian_to_joints(ee_pts, q_approach_part)
        segments.append({
            "name": "approach_to_grasp",
            "path_type": "LINEAR",
            "ee_positions": ee_pts,
            "joint_configs": jc,
        })

        # 3. grasp_part → lift  [LINEAR, straight up]
        ee_pts = self.linear_segment(p(q_grasp_part), p(q_lift), n_points=20)
        jc = self.cartesian_to_joints(ee_pts, q_grasp_part)
        segments.append({
            "name": "grasp_to_lift",
            "path_type": "LINEAR",
            "ee_positions": ee_pts,
            "joint_configs": jc,
        })

        # 4. lift → approach_chuck  [joint space]
        jc, ee_pts = joint_seg_with_ee(q_lift, q_approach_chuck)
        segments.append({
            "name": "lift_to_approach_chuck",
            "path_type": "JOINT",
            "ee_positions": ee_pts,
            "joint_configs": jc,
        })

        # 5. approach_chuck → insert  [LINEAR, straight into chuck axis]
        ee_pts = self.linear_segment(p(q_approach_chuck), p(q_insert), n_points=20)
        jc = self.cartesian_to_joints(ee_pts, q_approach_chuck)
        segments.append({
            "name": "approach_to_insert",
            "path_type": "LINEAR",
            "ee_positions": ee_pts,
            "joint_configs": jc,
        })

        # 6. insert → retract  [LINEAR, straight back out]
        ee_pts = self.linear_segment(p(q_insert), p(q_retract), n_points=20)
        jc = self.cartesian_to_joints(ee_pts, q_insert)
        segments.append({
            "name": "insert_to_retract",
            "path_type": "LINEAR",
            "ee_positions": ee_pts,
            "joint_configs": jc,
        })

        # 7. retract → home  [joint space]
        jc, ee_pts = joint_seg_with_ee(q_retract, q_home)
        segments.append({
            "name": "retract_to_home",
            "path_type": "JOINT",
            "ee_positions": ee_pts,
            "joint_configs": jc,
        })

        return segments

    def check_joint_limit_violations(self, joint_configs):
        """Return list of indices where any joint exceeds its limit."""
        violations = []
        for k, q in enumerate(joint_configs):
            for j, (lo, hi) in enumerate(JOINT_LIMITS):
                if q[j] < lo - 1e-6 or q[j] > hi + 1e-6:
                    violations.append((k, j, float(q[j])))
        return violations


# Linearity metric ──────────────────────────────────────────────────────── #
def linearity_error_mm(ee_pts):
    """
    Max perpendicular deviation from the straight line p0→pN (millimetres).
    """
    p0, pN = ee_pts[0], ee_pts[-1]
    d = pN - p0
    d_len = np.linalg.norm(d)
    if d_len < 1e-9:
        return 0.0
    d_hat = d / d_len
    max_err = 0.0
    for p in ee_pts:
        v = p - p0
        proj = np.dot(v, d_hat) * d_hat
        err = np.linalg.norm(v - proj)
        if err > max_err:
            max_err = err
    return max_err * 1000.0  # convert to mm


# Main: run, plot, save ─────────────────────────────────────────────────── #
def main():
    planner = CartesianPathPlanner()
    print("Planning load cycle …")
    segments = planner.plan_load_cycle()

    # Gather all EE positions and joint configs ─────────────────────────── #
    all_ee = np.vstack([s["ee_positions"] for s in segments])
    all_jc = np.vstack([s["joint_configs"] for s in segments])

    # Joint limit violations ─────────────────────────────────────────────── #
    violations = planner.check_joint_limit_violations(all_jc)

    # Linearity errors on LINEAR segments ──────────────────────────────── #
    lin_errs = []
    for s in segments:
        if s["path_type"] == "LINEAR":
            err = linearity_error_mm(s["ee_positions"])
            lin_errs.append(err)
            print(f"  {s['name']:30s} linearity error: {err:.4f} mm")

    max_lin_err = max(lin_errs) if lin_errs else 0.0

    # Plot ──────────────────────────────────────────────────────────────── #
    colors = {"LINEAR": "tab:red", "JOINT": "tab:blue"}
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    ax = axes[0]
    ax.set_title("End-effector Cartesian path (side view: X vs Z)")
    for s in segments:
        pts = s["ee_positions"]
        if isinstance(pts, np.ndarray) and pts.ndim == 2:
            ax.plot(pts[:, 0], pts[:, 2],
                    color=colors[s["path_type"]], linewidth=2,
                    label=s["path_type"])
            ax.scatter(pts[0, 0], pts[0, 2], color="green", s=40, zorder=5)
            ax.scatter(pts[-1, 0], pts[-1, 2], color="orange", s=40, zorder=5)
    handles = [plt.Line2D([0], [0], color=v, lw=2, label=k)
               for k, v in colors.items()]
    ax.legend(handles=handles)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Z (m)")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")

    ax2 = axes[1]
    ax2.set_title("Joint trajectories through load cycle")
    jnames = ["J1 base", "J2 shoulder", "J3 elbow", "J4 wrist", "J5 roll"]
    for j in range(5):
        ax2.plot(all_jc[:, j], label=jnames[j])
    ax2.set_xlabel("Trajectory sample index")
    ax2.set_ylabel("Joint angle (rad)")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    out_png = Path(__file__).parent / "results" / "cartesian_paths.png"
    out_png.parent.mkdir(exist_ok=True)
    fig.savefig(out_png, dpi=120)
    print(f"Saved {out_png}")

    # JSON results ──────────────────────────────────────────────────────── #
    result = {
        "n_segments": len(segments),
        "total_trajectory_points": int(len(all_ee)),
        "max_joint_limit_violations": len(violations),
        "max_linearity_error_mm": float(round(max_lin_err, 5)),
        "linear_segment_errors_mm": [float(round(e, 5)) for e in lin_errs],
        "script": "analysis/cartesian_path_planner.py",
    }
    out_json = Path(__file__).parent / "results" / "cartesian_plan.json"
    with open(out_json, "w") as f:
        json.dump(result, f, indent=2)
    print(f"Saved {out_json}")

    print("\n=== Cartesian Path Results ===")
    print(f"  Segments          : {result['n_segments']}")
    print(f"  Total path points : {result['total_trajectory_points']}")
    print(f"  JL violations     : {result['max_joint_limit_violations']}")
    print(f"  Max linearity err : {result['max_linearity_error_mm']:.4f} mm")


if __name__ == "__main__":
    main()
