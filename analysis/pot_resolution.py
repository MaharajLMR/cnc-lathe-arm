"""
How finely each knob can position its axis.

Three things limit it: the ADC (16-bit ADS1115, about 26400 counts over
3.3 V), how much of the pot's rotation we use, and the deadband in
pots.PotFilter that stops the arm twitching on noise. Prints a small table.

    python3 analysis/pot_resolution.py
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lathe_arm_teach"))
from lathe_arm_teach import arm_model as m, pots  # noqa: E402

COUNTS = 26400          # ADS1115 counts at 3.3 V, gain 1
NOISE_COUNTS = 8        # peak-to-peak noise we'd expect on a wiper with long leads


def axes():
    """(name, unit, full range of the axis in that unit)."""
    lim = m.LIMITS
    return [
        ("base", "deg", math.degrees(lim["j1_base_yaw"][1] - lim["j1_base_yaw"][0])),
        ("boom actuator", "mm", 1000 * (m.ACT_MAX - m.ACT_MIN)),
        ("wrist pitch", "deg", math.degrees(lim["j3_wrist_pitch"][1] - lim["j3_wrist_pitch"][0])),
        ("wrist roll", "deg", math.degrees(lim["j4_wrist_roll"][1] - lim["j4_wrist_roll"][0])),
        ("gripper", "mm", 2000 * 0.035),
    ]


def main():
    db = pots.PotFilter().deadband
    print(f"{'axis':14s} {'range':>10s} {'per count':>10s} {'noise':>8s} {'deadband':>9s}")
    for name, unit, span in axes():
        print(f"{name:14s} {span:7.0f} {unit:3s}{span / COUNTS:8.3f} {unit:3s}"
              f"{span * NOISE_COUNTS / COUNTS:6.2f} {span * db:7.2f}")
    print(f"\ndeadband is {db:.3f} of travel = {db * COUNTS:.0f} counts, so noise "
          f"({NOISE_COUNTS} counts) never gets through but a small turn of the knob does.")
    boom_mm = 1000 * (m.ACT_MAX - m.ACT_MIN) * db
    tip_mm = 1000 * m.BOOM * (m.boom_angle(m.ACT_MIN + boom_mm / 1000 + 0.2) - m.boom_angle(m.ACT_MIN + 0.2))
    print(f"one deadband step on the boom knob moves the gripper about {tip_mm:.1f} mm up or down.")


if __name__ == "__main__":
    main()
