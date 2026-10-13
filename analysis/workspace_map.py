"""
Workspace visualisation for the 5-DOF CNC lathe arm.

Produces two plots saved in analysis/results/:
  1. workspace_side.png  — YZ cross-section at q1=0 (2D reachability cloud)
  2. workspace_top.png   — XY top-view at z=0.15 m slice
  3. manipulability.png  — manipulability index along a load cycle path

Usage:
  python3 analysis/workspace_map.py

No ROS required. Uses only numpy + matplotlib.
"""

import math
import random
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize

# Add analysis/ to path so fk_kinematics can be imported directly
sys.path.insert(0, os.path.dirname(__file__))
from fk_kinematics import ee_position, JOINT_LIMITS, manipulability, L1, L2, L3, L4, L5

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "results")
os.makedirs(RESULTS_DIR, exist_ok=True)

random.seed(42)
np.random.seed(42)


# ── 1. Random workspace sample ────────────────────────────────────────────── #

N_SAMPLES = 40_000
print(f"Sampling {N_SAMPLES} random configurations …")

xs, ys, zs = [], [], []
ms = []

for _ in range(N_SAMPLES):
    q = [random.uniform(lo, hi) for lo, hi in JOINT_LIMITS]
    pos = ee_position(q)
    xs.append(pos[0])
    ys.append(pos[1])
    zs.append(pos[2])
    ms.append(manipulability(q))

xs = np.array(xs)
ys = np.array(ys)
zs = np.array(zs)
ms = np.array(ms)

reach = math.sqrt((L1 + L2 + L3 + L4 + L5) ** 2)
print(f"Max theoretical reach: {reach:.3f} m")
print(f"Actual sampled X range: [{xs.min():.3f}, {xs.max():.3f}]")
print(f"Actual sampled Z range: [{zs.min():.3f}, {zs.max():.3f}]")


# ── 2. Side view (XZ at q1=0) ─────────────────────────────────────────────── #

# Filter for near-planar (|y| < 0.05) to get the sagittal cross-section
mask_side = np.abs(ys) < 0.05
fig, ax = plt.subplots(figsize=(8, 7))
sc = ax.scatter(
    xs[mask_side], zs[mask_side],
    c=ms[mask_side], cmap="plasma", s=1, alpha=0.4,
    vmin=0, vmax=np.percentile(ms[mask_side], 95)
)
plt.colorbar(sc, ax=ax, label="Manipulability index")
ax.set_xlabel("X (m)")
ax.set_ylabel("Z (m)")
ax.set_title("5-DOF Lathe Arm — Side View Workspace (|y| < 0.05 m)")
ax.set_aspect("equal")
ax.grid(True, alpha=0.3)
fig.tight_layout()
out = os.path.join(RESULTS_DIR, "workspace_side.png")
fig.savefig(out, dpi=150)
plt.close(fig)
print(f"Saved: {out}")


# ── 3. Top view (XY at 0.10 < z < 0.20) ──────────────────────────────────── #

mask_top = (zs > 0.05) & (zs < 0.50)
fig, ax = plt.subplots(figsize=(8, 8))
sc = ax.scatter(
    xs[mask_top], ys[mask_top],
    c=ms[mask_top], cmap="viridis", s=1, alpha=0.4,
    vmin=0, vmax=np.percentile(ms[mask_top], 95)
)
plt.colorbar(sc, ax=ax, label="Manipulability index")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("5-DOF Lathe Arm — Top View Workspace (0.10 < z < 0.25 m)")
ax.set_aspect("equal")
ax.grid(True, alpha=0.3)
fig.tight_layout()
out = os.path.join(RESULTS_DIR, "workspace_top.png")
fig.savefig(out, dpi=150)
plt.close(fig)
print(f"Saved: {out}")


# ── 4. Manipulability along the example load/unload cycle ─────────────────── #

# Waypoints from example_load_cycle.yaml (in radians, converted from deg)
load_cycle_waypoints = [
    ("home",           [0.00,  0.00,  0.00,  0.00, 0.0]),
    ("approach_part",  [0.00,  0.80, -1.50,  0.50, 0.0]),
    ("grasp_part",     [0.00,  0.90, -1.70,  0.60, 0.0]),
    ("lift",           [0.00,  0.50, -1.20,  0.40, 0.0]),
    ("approach_chuck", [1.20,  0.60, -1.40,  0.40, 0.0]),
    ("insert",         [1.20,  0.65, -1.50,  0.45, 0.0]),
    ("retract",        [1.20,  0.50, -1.30,  0.35, 0.0]),
    ("home",           [0.00,  0.00,  0.00,  0.00, 0.0]),
]

manips = [manipulability(q) for _, q in load_cycle_waypoints]
labels = [name for name, _ in load_cycle_waypoints]

fig, ax = plt.subplots(figsize=(10, 4))
ax.bar(range(len(manips)), manips, color=["#e74c3c" if m < 0.01 else "#2ecc71" for m in manips])
ax.set_xticks(range(len(labels)))
ax.set_xticklabels(labels, rotation=20, ha="right")
ax.set_ylabel("Manipulability index")
ax.set_title("Manipulability at Each Load/Unload Cycle Waypoint")
ax.axhline(0.01, color="red", linestyle="--", linewidth=0.8, label="Singularity threshold")
ax.legend()
fig.tight_layout()
out = os.path.join(RESULTS_DIR, "manipulability.png")
fig.savefig(out, dpi=150)
plt.close(fig)
print(f"Saved: {out}")

print()
print("Waypoint manipulability summary:")
for name, m in zip(labels, manips):
    flag = " ⚠ near-singular" if m < 0.01 else ""
    print(f"  {name:20s}  {m:.4f}{flag}")
