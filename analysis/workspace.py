"""
Where the gripper can reach, side and top view, with the part stand and the
lathe chuck from the sim cell marked.

The dark line is the reach with the gripper kept vertical (wrist pitch
cancels the boom angle), which is how the arm is normally driven: one knob
then moves the gripper along that arc. The shaded area is everything the
fingertips can reach if the wrist pitch is free.

    python3 analysis/workspace.py
"""

import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lathe_arm_teach"))
from lathe_arm_teach import arm_model as m  # noqa: E402

OUT = Path(__file__).parent / "results"
STAND = (0.855, -0.599, 0.793)   # grip point on the part stand (lathe_cell.sdf)
CHUCK = (0.614, 0.877, 0.858)    # grip point when the part sits in the chuck


def sweep(n=120):
    lo2, hi2 = m.LIMITS["j2_boom_lift"]
    lo3, hi3 = m.LIMITS["j3_wrist_pitch"]
    pts = []
    for i in range(n + 1):
        q2 = lo2 + (hi2 - lo2) * i / n
        for k in range(n + 1):
            q3 = lo3 + (hi3 - lo3) * k / n
            x, _, z = m.tool_position([0, q2, q3, 0])
            pts.append((x, z))
    return pts


def level_arc(n=100):
    lo, hi = m.LIMITS["j2_boom_lift"]
    return [m.tool_position([0, q, m.level_wrist(q), 0]) for q in (lo + (hi - lo) * i / n for i in range(n + 1))]


def main():
    pts = sweep()
    arc = level_arc()
    r_level = [p[0] for p in arc]
    print(f"gripper vertical: radius {min(r_level):.2f} to {max(r_level):.2f} m, "
          f"height {min(p[2] for p in arc):.2f} to {max(p[2] for p in arc):.2f} m")
    print(f"wrist free:       radius up to {max(p[0] for p in pts):.2f} m, "
          f"height {min(p[1] for p in pts):.2f} to {max(p[1] for p in pts):.2f} m")
    for name, (x, y, z) in (("part stand", STAND), ("chuck", CHUCK)):
        print(f"{name:11s} r {math.hypot(x, y):.3f} m, yaw {math.degrees(math.atan2(y, x)):.0f} deg, "
              f"z {z:.3f} m, reachable: {m.inverse(x, y, z) is not None}")

    fig, (side, top) = plt.subplots(1, 2, figsize=(11, 5))
    side.scatter([p[0] for p in pts], [p[1] for p in pts], s=1, color="0.8", label="wrist free")
    side.plot(r_level, [p[2] for p in arc], color="k", lw=2, label="gripper vertical")
    side.plot([0, 0], [0, 1.6], color="#d9a800", lw=8, solid_capstyle="butt")
    for name, (x, y, z), c in (("stand", STAND, "tab:blue"), ("chuck", CHUCK, "tab:green")):
        side.plot(math.hypot(x, y), z, "o", color=c, label=name)
    side.set_xlabel("radius from column (m)")
    side.set_ylabel("height (m)")
    side.set_title("Side view")
    side.set_aspect("equal")
    side.set_xlim(-0.3, 1.6)
    side.set_ylim(0, 2.3)
    side.legend(loc="upper left", fontsize=8)
    side.grid(alpha=0.3)

    lo1, hi1 = m.LIMITS["j1_base_yaw"]
    rmax = max(p[0] for p in pts)
    for r, style in ((min(r_level), "k--"), (max(r_level), "k--"), (rmax, "0.6")):
        t = [lo1 + (hi1 - lo1) * i / 200 for i in range(201)]
        top.plot([r * math.cos(a) for a in t], [r * math.sin(a) for a in t], style, lw=1)
    top.plot(0, 0, "o", color="#d9a800", ms=14)
    top.plot(STAND[0], STAND[1], "s", color="tab:blue", ms=10, label="stand")
    top.plot(CHUCK[0], CHUCK[1], "o", color="tab:green", ms=10, label="chuck")
    top.set_title("Top view (base turns +-170 deg)")
    top.set_aspect("equal")
    top.set_xlabel("x (m)")
    top.set_ylabel("y (m)")
    top.legend(fontsize=8)
    top.grid(alpha=0.3)
    fig.tight_layout()
    OUT.mkdir(exist_ok=True)
    fig.savefig(OUT / "workspace.png", dpi=110)


if __name__ == "__main__":
    main()
