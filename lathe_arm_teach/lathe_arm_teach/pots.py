"""
Turning potentiometer readings into joint targets.

On the robot each axis had a knob. A reading in 0..1 (fraction of the pot's
travel) is mapped to the axis range; the boom knob maps to actuator stroke,
like the real actuator, and is converted to a boom angle. Readings are
smoothed and small changes ignored so a noisy pot doesn't make the arm jitter.

Pure Python so it can be tested without hardware or ROS.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional

from . import arm_model


@dataclass
class Channel:
    """Calibration for one pot: raw readings at the two ends of its travel."""
    raw_min: float = 0.0
    raw_max: float = 1.0
    invert: bool = False

    def fraction(self, raw: float) -> float:
        span = self.raw_max - self.raw_min
        f = 0.0 if span == 0 else (raw - self.raw_min) / span
        f = min(1.0, max(0.0, f))
        return 1.0 - f if self.invert else f


def lerp(lo, hi, f):
    return lo + (hi - lo) * f


def fractions_to_joints(f: List[float], keep_level: bool = False) -> List[float]:
    """Five pot fractions -> [j1, j2, j3, j4, j5].

    f[1] is the actuator: 0 = fully retracted, 1 = fully extended.
    With keep_level the wrist pitch knob is ignored and the gripper is held
    vertical, which is how we usually drove it.
    """
    lim = arm_model.LIMITS
    j1 = lerp(*lim["j1_base_yaw"], f[0])
    j2 = arm_model.boom_angle(lerp(arm_model.ACT_MIN, arm_model.ACT_MAX, f[1]))
    j3 = arm_model.level_wrist(j2) if keep_level else lerp(*lim["j3_wrist_pitch"], f[2])
    j4 = lerp(*lim["j4_wrist_roll"], f[3])
    j5 = lerp(0.0, 0.035, f[4])
    return [j1, j2, j3, j4, j5]


def joints_to_fractions(q: List[float]) -> List[float]:
    """Inverse of fractions_to_joints (used to start the web sliders where the arm is)."""
    lim = arm_model.LIMITS

    def frac(v, lo, hi):
        return min(1.0, max(0.0, (v - lo) / (hi - lo)))

    act = arm_model.actuator_length(q[1])
    return [frac(q[0], *lim["j1_base_yaw"]),
            frac(act, arm_model.ACT_MIN, arm_model.ACT_MAX),
            frac(q[2], *lim["j3_wrist_pitch"]),
            frac(q[3], *lim["j4_wrist_roll"]),
            frac(q[4], 0.0, 0.035)]


@dataclass
class PotFilter:
    """Exponential smoothing plus a deadband, per channel."""
    alpha: float = 0.3          # 1 = no smoothing
    deadband: float = 0.004     # fraction of travel
    state: Optional[List[float]] = None
    last_sent: Optional[List[float]] = field(default=None)

    def update(self, f: List[float]) -> List[float]:
        if self.state is None:
            self.state = list(f)
        else:
            self.state = [s + self.alpha * (x - s) for s, x in zip(self.state, f)]
        return list(self.state)

    def changed(self, f: List[float]) -> bool:
        """True if any channel moved more than the deadband since the last send."""
        if self.last_sent is None or any(abs(a - b) > self.deadband for a, b in zip(f, self.last_sent)):
            self.last_sent = list(f)
            return True
        return False


def ads1115_to_fraction(raw: int, vref_counts: int = 26400) -> float:
    """ADS1115 reading (gain 1, pot across 3.3 V) to 0..1. 3.3 V reads ~26400 counts."""
    return min(1.0, max(0.0, raw / vref_counts))


def degrees(q: List[float]) -> List[float]:
    return [math.degrees(v) for v in q[:4]] + [q[4] * 2000.0]   # gripper as opening in mm
