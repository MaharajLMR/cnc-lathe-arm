#!/usr/bin/env python3
"""Work out the load cycle from the cell geometry and write the waypoints file.

I used to type these joint angles in by hand and nudge them until the gripper
lined up. Now the cell dimensions live in lathe_arm_teach/cell.py and this
solves for the joints, so moving the lathe means changing one number.

It also prints the poses the world file needs, since lathe_cell.sdf has to put
the tray and the machine exactly where this thinks they are.

Usage:
    python3 tools/make_waypoints.py            # print only
    python3 tools/make_waypoints.py --write    # update the yaml too
"""

import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "lathe_arm_teach"))

from lathe_arm_teach import arm_model as am      # noqa: E402
from lathe_arm_teach import cell                 # noqa: E402

OUT = ROOT / "lathe_arm_teach/waypoints/example_load_cycle.yaml"


def solve(point, roll, grip, name):
    q = am.inverse(*point, pitch_hint=0.0)
    if q is None:
        raise SystemExit(f"{name}: {point} is out of reach")
    joints = [q[0], q[1], q[2], roll, grip]
    if not am.within_limits(joints):
        raise SystemExit(f"{name}: {joints} is outside the joint limits")
    err = math.dist(am.tool_position(joints), point)
    if err > 1e-4:
        raise SystemExit(f"{name}: solved to {err:.4f} m away")
    return joints


def cycle():
    o, c = cell.OPEN, cell.CLOSED
    flat, roll = 0.0, cell.ROLL_FOR_CHUCK
    home = [0.0, 0.1745, -0.1745, 0.0, o]

    over_tray = cell.level_point(cell.TRAY_YAW, cell.PICK_TIP_Z + cell.LIFT)
    over_machine = cell.level_point(cell.LINEUP_YAW, cell.CHUCK_TIP_Z + cell.CLEAR)

    return [
        ("home", home),
        ("above_billet", solve(over_tray, flat, o, "above_billet")),
        ("at_billet", solve(cell.PICK, flat, o, "at_billet")),
        ("grip", solve(cell.PICK, flat, c, "grip")),
        ("lift", solve(over_tray, flat, c, "lift")),
        # a quarter turn of the wrist puts the billet along the spindle
        ("turn_to_spindle", solve(over_tray, roll, c, "turn_to_spindle")),
        ("over_machine", solve(over_machine, roll, c, "over_machine")),
        ("on_spindle_axis", solve(cell.LINEUP, roll, c, "on_spindle_axis")),
        ("into_chuck", solve(cell.CHUCK, roll, c, "into_chuck")),
        ("release", solve(cell.CHUCK, roll, o, "release")),
        ("withdraw", solve(cell.LINEUP, roll, o, "withdraw")),
        ("clear_machine", solve(over_machine, roll, o, "clear_machine")),
        ("home", home),
    ]


def yaml_text(points):
    head = [
        "# Load one billet from the job tray into the lathe chuck.",
        "# Joints: base yaw, boom lift, wrist pitch, wrist roll (rad), gripper (m per finger).",
        f"# Gripper {cell.OPEN} = open, {cell.CLOSED} = closed on the {cell.BILLET_D * 1000:.0f} mm billet.",
        "#",
        "# Written by tools/make_waypoints.py from lathe_arm_teach/cell.py. Do not",
        "# edit by hand: change the cell geometry and run it again.",
        "",
        "waypoints:",
    ]
    for name, q in points:
        head.append(f"  - name: {name}")
        head.append("    joints: [" + ", ".join(f"{v:.4f}" for v in q) + "]")
    return "\n".join(head) + "\n"


def report(points):
    print("cell geometry")
    print(f"  tray     yaw {math.degrees(cell.TRAY_YAW):7.2f} deg  r {cell.R_PICK:.4f}  "
          f"billet axis z {cell.BILLET_Z:.4f}  tips z {cell.PICK_TIP_Z:.4f}")
    print(f"  lathe    yaw {math.degrees(cell.LATHE_YAW):7.2f} deg  r {cell.R_CHUCK:.4f}  "
          f"spindle z {cell.SPINDLE_Z:.4f}  tips z {cell.CHUCK_TIP_Z:.4f}")
    print()
    print("poses for lathe_cell.sdf")
    print(f"  tray  <pose>{cell.TRAY_ORIGIN[0]:.4f} {cell.TRAY_ORIGIN[1]:.4f} 0 0 0 "
          f"{cell.TRAY_MODEL_YAW:.4f}</pose>")
    print(f"  lathe <pose>{cell.LATHE_ORIGIN[0]:.4f} {cell.LATHE_ORIGIN[1]:.4f} 0 0 0 "
          f"{cell.LATHE_MODEL_YAW:.4f}</pose>")
    print(f"  part  <pose>{cell.BILLET[0]:.4f} {cell.BILLET[1]:.4f} {cell.BILLET[2]:.4f} "
          f"0 1.5708 {cell.TRAY_YAW:.4f}</pose>")
    print(f"  part_grasp PICK  = {tuple(round(v, 4) for v in cell.PICK)}")
    print(f"  part_grasp CHUCK = {tuple(round(v, 4) for v in cell.CHUCK)}")
    print()
    print("waypoints")
    for name, q in points:
        tip = am.tool_position(q)
        pitch = math.degrees(q[1] + q[2])
        print(f"  {name:16s} tips ({tip[0]:+.3f} {tip[1]:+.3f} {tip[2]:+.3f})  "
              f"yaw {math.degrees(q[0]):+7.2f}  gripper pitch {pitch:+6.2f} deg")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()

    points = cycle()
    report(points)
    if args.write:
        OUT.write_text(yaml_text(points))
        print(f"\nwrote {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
