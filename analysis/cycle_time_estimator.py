"""
Estimated load/unload cycle time for the 5-DOF CNC lathe arm.

Uses the FK module + trapezoidal velocity profiles to:
  1. Compute straight-line Cartesian distance between each waypoint.
  2. Fit a trapezoidal velocity profile to each segment.
  3. Sum up to get total estimated cycle time.
  4. Overlay with manipulability at each waypoint.

⚠ Assumes placeholder dimensions. Update when Maharaj provides real measurements.

Output: analysis/results/cycle_estimate.png + cycle_estimate.txt

Usage:
  python3 analysis/cycle_time_estimator.py
"""

import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(__file__))
from fk_kinematics import ee_position, manipulability, JOINT_LIMITS

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "results")
os.makedirs(RESULTS_DIR, exist_ok=True)

# ── Cycle waypoints — joint angles from example_load_cycle.yaml ───────────── #
WAYPOINTS = [
    ("home",           [0.00,  0.00,  0.00,  0.00, 0.0]),
    ("approach_part",  [0.00,  0.80, -1.50,  0.50, 0.0]),
    ("grasp_part",     [0.00,  0.90, -1.70,  0.60, 0.0]),
    ("lift",           [0.00,  0.50, -1.20,  0.40, 0.0]),
    ("approach_chuck", [1.20,  0.60, -1.40,  0.40, 0.0]),
    ("insert",         [1.20,  0.65, -1.50,  0.45, 0.0]),
    ("retract",        [1.20,  0.50, -1.30,  0.35, 0.0]),
    ("home",           [0.00,  0.00,  0.00,  0.00, 0.0]),
]

# ── Motion limits ────────────────────────────────────────────────────────── #
# Conservative: 1.0 m/s Cartesian peak, 4 m/s² accel (no S-curve rounding)
V_MAX  = 0.80   # m/s
A_MAX  = 3.00   # m/s²
DWELL  = 0.20   # s at each waypoint (open/close gripper, settle time)


def trapezoid_time(d: float, v_max: float = V_MAX, a_max: float = A_MAX) -> float:
    """Time to travel distance d with a trapezoidal velocity profile."""
    if d < 1e-6:
        return 0.0
    v_peak = min(v_max, math.sqrt(a_max * d))
    t_ramp  = v_peak / a_max
    d_ramp  = 0.5 * a_max * t_ramp ** 2
    d_cruise = max(0.0, d - 2 * d_ramp)
    t_cruise = d_cruise / v_peak
    return 2 * t_ramp + t_cruise


def estimate():
    positions = [ee_position(q) for _, q in WAYPOINTS]
    manips    = [manipulability(q) for _, q in WAYPOINTS]
    names     = [n for n, _ in WAYPOINTS]

    segments = []
    total_t  = 0.0
    for i in range(len(WAYPOINTS) - 1):
        dist = float(np.linalg.norm(positions[i + 1] - positions[i]))
        t_move = trapezoid_time(dist)
        t_seg  = t_move + DWELL
        segments.append({
            "name":   f"{names[i]}→{names[i+1]}",
            "dist":   dist,
            "t_move": t_move,
            "t_total": t_seg,
        })
        total_t += t_seg

    return segments, total_t, names, manips


def plot(segments, total_t, names, manips):
    fig, axes = plt.subplots(2, 1, figsize=(11, 8))

    labels = [s["name"] for s in segments]
    times  = [s["t_total"] for s in segments]
    colors = ["#e74c3c" if m < 0.01 else "#3498db" for m in manips[:-1]]

    axes[0].barh(labels, times, color=colors)
    axes[0].set_xlabel("Segment time (s)  [move + 0.20 s dwell]")
    axes[0].set_title(f"Load Cycle Segment Times — estimated total: {total_t:.2f} s")
    axes[0].axvline(0.5, color="green", linestyle="--", linewidth=0.8, label="0.5 s target")
    axes[0].legend()
    axes[0].grid(True, axis="x", alpha=0.3)

    ax2 = axes[1]
    xs = range(len(names))
    bar_colors = ["#e74c3c" if m < 0.01 else "#2ecc71" for m in manips]
    ax2.bar(xs, manips, color=bar_colors, width=0.6)
    ax2.set_xticks(list(xs))
    ax2.set_xticklabels(names, rotation=20, ha="right")
    ax2.set_ylabel("Manipulability index")
    ax2.set_title("Manipulability at Each Waypoint (red = singular)")
    ax2.axhline(0.01, color="red", linestyle="--", linewidth=0.8, label="threshold 0.01")
    ax2.legend()
    ax2.grid(True, axis="y", alpha=0.3)

    fig.tight_layout()
    out = os.path.join(RESULTS_DIR, "cycle_estimate.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def write_report(segments, total_t):
    lines = [
        "5-DOF Lathe Arm — Cycle Time Estimate",
        "=" * 55,
        f"v_max={V_MAX} m/s  a_max={A_MAX} m/s²  dwell={DWELL} s/waypoint",
        f"(industrial dimensions: L1=0.18 L2=0.35 L3=0.30 L4=0.09 L5=0.065 m)",
        "",
        f"{'Segment':25s} {'Dist (m)':>9} {'Move (s)':>9} {'Total (s)':>10}",
        "-" * 57,
    ]
    for seg in segments:
        lines.append(
            f"{seg['name']:25s} {seg['dist']:9.4f} {seg['t_move']:9.3f} {seg['t_total']:10.3f}"
        )
    lines.extend([
        "-" * 57,
        f"{'Estimated cycle time':25s} {'':>9} {'':>9} {total_t:10.3f} s",
        "",
        "Note: 'home' waypoint is singular (arm fully extended along X).",
        "      Design a non-zero home if this is a robot restart position.",
    ])
    txt = "\n".join(lines)
    out = os.path.join(RESULTS_DIR, "cycle_estimate.txt")
    with open(out, "w") as f:
        f.write(txt + "\n")
    return out, txt


if __name__ == "__main__":
    segs, total, names, manips = estimate()
    png = plot(segs, total, names, manips)
    txt_path, report = write_report(segs, total)
    print(report)
    print(f"\nSaved: {png}")
    print(f"Saved: {txt_path}")
