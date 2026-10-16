"""
Fault detector for the 5-DOF CNC lathe loading arm.

Monitors a joint trajectory for five fault types:
  E01, joint position limit violation
  E02, joint velocity exceeded rated limit
  E03, tracking error > 1° (simulated ±0.5% sensor noise)
  E04, near-singular configuration (manipulability μ < 0.01)
  E05, end-effector entered the forbidden operator safety zone

Usage:
  python3 analysis/fault_detector.py
"""

import sys
import json
import math
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from fk_kinematics import JOINT_LIMITS, ee_position, manipulability

# Limits ────────────────────────────────────────────────────────────────── #
JOINT_VEL_LIMIT_RAD_S = [2.0, 2.0, 2.0, 3.0, 4.0]
TRACKING_ERROR_LIMIT_RAD = math.radians(1.0)
SINGULARITY_MU_THRESHOLD = 0.01

# Forbidden zone: region where the human operator stands (behind / beside the arm)
SAFETY_ZONE = {
    "x_min": -0.40, "x_max": 0.20,
    "y_min": -0.10, "y_max": 0.10,
    "z_min":  0.50, "z_max": 1.20,
}


class FaultDetector:
    """
    Stateless fault checker: pass a trajectory, get back a list of fault events.
    """

    def simulate_noise(self, joint_configs: np.ndarray, noise_pct: float = 0.5,
                       seed: int = 0) -> np.ndarray:
        """Add ±noise_pct% Gaussian noise to simulate encoder/tracking error."""
        rng = np.random.RandomState(seed)
        noise = rng.normal(0, noise_pct / 100.0, joint_configs.shape)
        return joint_configs + noise * np.abs(joint_configs + 1e-6)

    def check_trajectory(
        self,
        times: np.ndarray,
        joint_configs: np.ndarray,
        joint_vels: np.ndarray | None = None,
    ) -> list[dict]:
        """
        Run all five checks on a trajectory.

        Args:
            times:         1-D array of timestamps (seconds)
            joint_configs: N×5 array of joint positions (radians)
            joint_vels:    N×5 array (optional). If None, estimated via finite diff.

        Returns:
            List of {time, fault_code, joint_index, value, description}.
        """
        n = len(times)
        if joint_vels is None:
            dt = np.diff(times, prepend=times[0] - (times[1] - times[0]) if n > 1 else 1e-3)
            dt = np.where(np.abs(dt) < 1e-9, 1e-9, dt)
            dvdt = np.diff(joint_configs, axis=0, prepend=joint_configs[:1])
            joint_vels = dvdt / dt[:, None]

        noisy = self.simulate_noise(joint_configs)
        faults = []

        for k in range(n):
            t = float(times[k])
            q = joint_configs[k]
            qv = joint_vels[k]
            qn = noisy[k]
            pos = ee_position(q)

            # E01, joint position limit
            for j, (lo, hi) in enumerate(JOINT_LIMITS):
                if q[j] < lo - 1e-4 or q[j] > hi + 1e-4:
                    faults.append({
                        "time_s": t, "fault_code": "E01",
                        "joint": j + 1,
                        "value_rad": float(q[j]),
                        "limit": f"[{lo:.3f}, {hi:.3f}]",
                        "description": f"J{j+1} = {math.degrees(q[j]):.1f}° outside limits",
                    })

            # E02, velocity
            for j, v_lim in enumerate(JOINT_VEL_LIMIT_RAD_S):
                if abs(qv[j]) > v_lim * 1.5:
                    faults.append({
                        "time_s": t, "fault_code": "E02",
                        "joint": j + 1,
                        "value_rad_s": float(qv[j]),
                        "limit_rad_s": v_lim * 1.5,
                        "description": f"J{j+1} velocity {math.degrees(qv[j]):.1f}°/s > limit",
                    })

            # E03, tracking error (commanded vs noisy)
            track_err = np.abs(qn - q)
            for j, err in enumerate(track_err):
                if err > TRACKING_ERROR_LIMIT_RAD:
                    faults.append({
                        "time_s": t, "fault_code": "E03",
                        "joint": j + 1,
                        "error_deg": float(math.degrees(err)),
                        "description": f"J{j+1} tracking error {math.degrees(err):.2f}° > 1°",
                    })

            # E04, singularity
            mu = manipulability(q)
            if mu < SINGULARITY_MU_THRESHOLD:
                faults.append({
                    "time_s": t, "fault_code": "E04",
                    "manipulability": float(mu),
                    "description": f"Near-singular: μ = {mu:.4f} < {SINGULARITY_MU_THRESHOLD}",
                })

            # E05, safety zone
            sz = SAFETY_ZONE
            if (sz["x_min"] <= pos[0] <= sz["x_max"] and
                    sz["y_min"] <= pos[1] <= sz["y_max"] and
                    sz["z_min"] <= pos[2] <= sz["z_max"]):
                faults.append({
                    "time_s": t, "fault_code": "E05",
                    "ee_xyz": [float(v) for v in pos],
                    "description": (f"EE ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) "
                                    f"entered operator safety zone"),
                })

        return faults

    def fault_free_rate(self, times, faults):
        """Fraction of timesteps that are fault-free."""
        fault_times = set(f["time_s"] for f in faults)
        clean = sum(1 for t in times if float(t) not in fault_times)
        return clean / len(times) if len(times) > 0 else 1.0


# Demo: run on the load-cycle trajectory ────────────────────────────────── #
def main():
    from cartesian_path_planner import CartesianPathPlanner

    print("Building load-cycle trajectory …")
    planner = CartesianPathPlanner()
    segments = planner.plan_load_cycle()

    all_jc = np.vstack([s["joint_configs"] for s in segments])
    n = len(all_jc)
    times = np.linspace(0.0, 5.55, n)

    detector = FaultDetector()
    faults = detector.check_trajectory(times, all_jc)
    ffr = detector.fault_free_rate(times, faults)

    # Summarise by code
    from collections import Counter
    counts = Counter(f["fault_code"] for f in faults)

    print(f"\nTotal trajectory points : {n}")
    print(f"Fault events detected   : {len(faults)}")
    print(f"Fault-free rate         : {ffr*100:.1f}%")
    print("\nBreakdown by fault code:")
    FAULT_DESC = {
        "E01": "Joint position limit violation",
        "E02": "Joint velocity exceeded rated limit",
        "E03": "Joint tracking error exceeded 1°",
        "E04": "Near-singular configuration (μ < 0.01)",
        "E05": "Safety zone violation, operator proximity",
    }
    for code in ["E01", "E02", "E03", "E04", "E05"]:
        print(f"  {code}  {counts.get(code, 0):4d}  {FAULT_DESC[code]}")

    # Save
    out_dir = Path(__file__).parent / "results"
    out_dir.mkdir(exist_ok=True)
    result = {
        "trajectory_points": n,
        "fault_events_total": len(faults),
        "fault_free_rate_pct": float(round(ffr * 100, 1)),
        "counts_by_code": {k: int(v) for k, v in counts.items()},
        "safety_zone": SAFETY_ZONE,
        "script": "analysis/fault_detector.py",
    }
    out = out_dir / "fault_report.json"
    with open(out, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\nSaved {out}")


if __name__ == "__main__":
    main()
