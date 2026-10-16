"""
Cycle time for a points file, two ways: every arm move given a fixed 3 s
(what replay_node did at first), and each move given only the time its
slowest joint needs at pot_teleop's speed limits, stretched by 1.9 so the
peak speed stays inside the limit (what replay_node does now). Gripper changes take 1 s either way.

    python3 analysis/cycle_time.py [points.yaml]
"""

import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "lathe_arm_teach"))
from lathe_arm_teach.pot_teleop import SPEED, move_time  # noqa: E402

REPLAY_MOVE = 3.0
GRIPPER_MOVE = 1.0
PEAK = 1.9
SETTLE = 0.2            # pause at each point before the next move


def segments(points):
    """(name, slowest-joint time, gripper changed) for each move in the file."""
    out = []
    for prev, cur in zip(points, points[1:]):
        a, b = prev["joints"], cur["joints"]
        arm = move_time(a[:4], b[:4], 0.0)
        out.append((cur["name"], arm, abs(a[4] - b[4]) > 1e-4))
    return out


def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "lathe_arm_teach/waypoints/example_load_cycle.yaml"
    points = yaml.safe_load(path.read_text())["waypoints"]
    replay = fast = 0.0
    print(f"{'to point':16s} {'speed-limited':>13s}  gripper")
    for name, arm, grip in segments(points):
        g = GRIPPER_MOVE if grip else 0.0
        replay += REPLAY_MOVE + g
        fast += (PEAK * max(arm, 0.5) if arm > 0 else 0.0) + g + SETTLE
        print(f"{name:16s} {arm:10.2f} s   {'yes' if grip else ''}")
    print(f"\nfixed 3 s per move:   {replay:.1f} s")
    print(f"speed-limited moves:  {fast:.1f} s  (joint speeds {SPEED} rad/s, x1.9)")


if __name__ == "__main__":
    main()
