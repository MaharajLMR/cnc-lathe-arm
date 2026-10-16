"""
How much stroke and force the boom actuator needs.

Uses the linkage in arm_model: the actuator pushes from a mount low on the
column to a point 0.35 m along the boom. For each boom angle it works out
the actuator length and the push needed to hold the boom, wrist, gripper
and a part still. Masses are estimates (same as the URDF), not weighed.

    python3 analysis/actuator_sizing.py
"""

import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lathe_arm_teach"))
from lathe_arm_teach import arm_model as m  # noqa: E402

G = 9.81
BOOM_MASS = 6.0         # kg, steel tube, centre of mass at mid-boom
TIP_MASS = 0.5          # kg, wrist servos + gripper
PART_MASS = 0.3         # kg, 50 mm aluminium bar
OUT = Path(__file__).parent / "results"


def moment_arm(boom):
    """Perpendicular distance from the boom pivot to the actuator's line of action."""
    ax = m.PIVOT_X + m.ACT_ON_BOOM * math.cos(boom)
    az = m.PIVOT_Z + m.ACT_ON_BOOM * math.sin(boom)
    dx, dz = ax - m.ACT_LOW[0], az - m.ACT_LOW[1]
    length = math.hypot(dx, dz)
    rx, rz = ax - m.PIVOT_X, az - m.PIVOT_Z      # pivot to the upper mount
    return abs(rx * dz - rz * dx) / length


def holding_force(boom, part=PART_MASS):
    """Actuator push (N) to hold the boom still at this angle."""
    torque = G * math.cos(boom) * (BOOM_MASS * m.BOOM / 2 + (TIP_MASS + part) * m.BOOM)
    return torque / moment_arm(boom)


def main():
    lo, hi = m.LIMITS["j2_boom_lift"]
    angles = [lo + (hi - lo) * i / 150 for i in range(151)]
    lengths = [1000 * m.actuator_length(a) for a in angles]
    forces = [holding_force(a) for a in angles]
    peak = max(forces)
    at = angles[forces.index(peak)]

    print(f"actuator length   {1000 * m.ACT_MIN:.0f} mm (boom {math.degrees(lo):.0f} deg) "
          f"to {1000 * m.ACT_MAX:.0f} mm (boom {math.degrees(hi):.0f} deg)")
    print(f"stroke needed     {1000 * (m.ACT_MAX - m.ACT_MIN):.0f} mm")
    print(f"moment arm        {1000 * min(map(moment_arm, angles)):.0f} to {1000 * max(map(moment_arm, angles)):.0f} mm")
    print(f"peak push         {peak:.0f} N at boom {math.degrees(at):.0f} deg (with a {PART_MASS} kg part)")
    print(f"empty gripper     {max(holding_force(a, 0) for a in angles):.0f} N")
    print(f"with 2x margin    pick an actuator rated {math.ceil(2 * peak / 100) * 100:.0f} N or more")

    deg = [math.degrees(a) for a in angles]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 4))
    a1.plot(deg, lengths)
    a1.set_xlabel("boom angle (deg)")
    a1.set_ylabel("actuator pin-to-pin length (mm)")
    a1.set_title("Actuator length")
    a1.grid(alpha=0.3)
    a2.plot(deg, forces, label=f"{PART_MASS} kg part")
    a2.plot(deg, [holding_force(a, 0) for a in angles], "--", label="empty")
    a2.set_xlabel("boom angle (deg)")
    a2.set_ylabel("push to hold (N)")
    a2.set_title("Actuator force")
    a2.legend()
    a2.grid(alpha=0.3)
    fig.tight_layout()
    OUT.mkdir(exist_ok=True)
    fig.savefig(OUT / "actuator.png", dpi=110)


if __name__ == "__main__":
    main()
