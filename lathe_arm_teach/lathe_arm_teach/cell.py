"""Where everything sits in the loading cell.

The world file, the waypoints, part_grasp and the tests all need the same
numbers, so they live here instead of being typed into each of them.

Frame: origin on the floor at the column axis, z up, same as arm_model.

The machine is a horizontal CNC lathe, which is what we actually loaded at the
hackathon: the spindle is horizontal and the billet goes into the chuck along
that axis. Standing at the arm and looking at the machine, the spindle runs
left to right, so the billet slides in sideways. That matters for the arm,
because the only motion it has that is horizontal and keeps the gripper level
is swinging the base: lifting the boom moves the gripper up and out together.
So the insertion is a base swing, and the billet has to be turned to lie along
that swing before it goes in.
"""

import math

from . import arm_model as am

BILLET_D = 0.050
BILLET_L = 0.100

# tool_position() returns the fingertips, but the billet is held across the
# middle of the fingers, so its axis sits this far above the tips
GRIP_RISE = 0.045

OPEN = 0.030
CLOSED = 0.0215        # on a 50 mm billet

# job tray
TRAY_YAW = math.radians(-35.0)
TRAY_TOP = 0.832
POCKETS = 5
POCKET_PITCH = 0.070
BILLET_Z = TRAY_TOP + BILLET_D / 2

# lathe
LATHE_YAW = math.radians(55.0)
SPINDLE_Z = 1.026
JAW_FACE = 0.010       # chuck face, along the spindle axis from the loaded billet centre
APPROACH = 0.130       # how far back along the axis the arm lines up before going in
LIFT = 0.200           # straight-up clearance after gripping
CLEAR = 0.350          # fingertip height for the swing across, above the front guard

# How tall the machine can be where the arm swings over it. The boom pivot is
# at 1.50 m and the gripper hangs about 0.30 m below the boom, so anything
# taller than this in the swept path gets hit. It is why the loading side of
# the machine is cut down and the tall parts are all behind the spindle or
# past the ends. A real cell would use a taller gantry instead.
GUARD_TOP = SPINDLE_Z - GRIP_RISE + CLEAR - 0.08


def level_pose(tip_z: float):
    """(boom angle, radius) for the fingertips at this height with the gripper level.

    With the gripper hanging straight down the wrist cancels the boom, so there
    is only one degree of freedom left and the tips trace a circle: the radius
    is fixed once you pick the height.
    """
    s = (tip_z - (am.PIVOT_Z - am.TOOL)) / am.BOOM
    if not -1.0 <= s <= 1.0:
        raise ValueError(f"{tip_z} m is outside the level-gripper range")
    q2 = math.asin(s)
    return q2, am.PIVOT_X + am.BOOM * math.cos(q2)


def polar(r: float, yaw: float, z: float):
    return (r * math.cos(yaw), r * math.sin(yaw), z)


def level_point(yaw: float, tip_z: float):
    """Fingertip point at this yaw and height with the gripper still level.

    Going straight up from a level pose tilts the wrist, because the boom and
    the wrist have to trade off against each other. Staying on this curve costs
    a little radius as the arm rises, but the billet stays flat the whole way.
    """
    return polar(level_pose(tip_z)[1], yaw, tip_z)


def tangent(yaw: float):
    """Unit vector the fingertips move along when only the base yaw changes."""
    return (-math.sin(yaw), math.cos(yaw), 0.0)


PICK_TIP_Z = BILLET_Z - GRIP_RISE
Q2_PICK, R_PICK = level_pose(PICK_TIP_Z)

CHUCK_TIP_Z = SPINDLE_Z - GRIP_RISE
Q2_CHUCK, R_CHUCK = level_pose(CHUCK_TIP_Z)

# fingertip grip point on the billet in the middle pocket
PICK = polar(R_PICK, TRAY_YAW, PICK_TIP_Z)
BILLET = polar(R_PICK, TRAY_YAW, BILLET_Z)          # billet centre on the tray

# fingertip point with the billet seated in the chuck, and the point the arm
# lines up at first, APPROACH back along the spindle axis
CHUCK = polar(R_CHUCK, LATHE_YAW, CHUCK_TIP_Z)
_t = tangent(LATHE_YAW)
AXIS_IN = _t                                         # direction the billet moves to go in
LINEUP = tuple(CHUCK[i] - APPROACH * _t[i] for i in range(3))
LINEUP_YAW = math.atan2(LINEUP[1], LINEUP[0])

# the lathe model's own frame: +x back along the spindle away from the chuck,
# +y radially out from the column, origin under the loaded billet centre
LATHE_ORIGIN = polar(R_CHUCK, LATHE_YAW, 0.0)
LATHE_MODEL_YAW = LATHE_YAW - math.pi / 2

# the tray model's frame: +x radially out, +y along the pocket row
TRAY_ORIGIN = polar(R_PICK, TRAY_YAW, 0.0)
TRAY_MODEL_YAW = TRAY_YAW

# billet lying in the tray points radially out, so with the wrist roll at zero
# the fingers close across it; rolling the wrist a quarter turn afterwards puts
# it along the spindle
ROLL_FOR_CHUCK = math.pi / 2 - 1e-4                  # stay just inside the servo limit
