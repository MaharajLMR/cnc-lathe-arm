"""
Geometry and kinematics of the jib arm we built for the RANE hackathon.

A yellow steel column on a slewing base, a boom hinged near the top of the
column and lifted by a linear actuator, and a two-servo wrist with a gripper
hanging from the boom tip. Five axes:

    j1  base yaw      motor + slewing bearing turn the whole column
    j2  boom lift     linear actuator between the column and the boom
    j3  wrist pitch   servo at the boom tip
    j4  wrist roll    servo above the gripper
    j5  gripper       servo opening the fingers (finger gap in metres)

Column height and boom length are what we measured on the robot; the
smaller dimensions are read off photos, so treat them as approximate.

Frame: origin on the floor at the column axis, z up, x along the boom when
j1 = 0. j2 = 0 is a horizontal boom, positive raises it. j3 is measured
relative to the boom; j2 + j3 = 0 keeps the gripper hanging straight down.
"""

import math

PIVOT_X = 0.13        # boom pivot, ahead of the column axis
PIVOT_Z = 1.50        # boom pivot height above the floor
BOOM = 1.00           # pivot to wrist pitch axis
ACT_LOW = (0.13, 0.62)   # actuator lower mount on the column (x, z)
ACT_ON_BOOM = 0.35    # actuator upper mount, distance from pivot along the boom
TOOL = 0.30           # wrist pitch axis to the gripper fingertips

JOINTS = ["j1_base_yaw", "j2_boom_lift", "j3_wrist_pitch", "j4_wrist_roll", "j5_gripper"]
LIMITS = {
    "j1_base_yaw": (-math.radians(170), math.radians(170)),
    "j2_boom_lift": (math.radians(-30), math.radians(45)),
    "j3_wrist_pitch": (math.radians(-135), math.radians(135)),
    "j4_wrist_roll": (-math.pi / 2, math.pi / 2),
    "j5_gripper": (0.0, 0.035),     # per finger
}


def actuator_length(boom_angle: float) -> float:
    """Pin-to-pin length of the linear actuator for a boom angle (rad)."""
    ax = PIVOT_X + ACT_ON_BOOM * math.cos(boom_angle)
    az = PIVOT_Z + ACT_ON_BOOM * math.sin(boom_angle)
    return math.hypot(ax - ACT_LOW[0], az - ACT_LOW[1])


ACT_MIN = actuator_length(LIMITS["j2_boom_lift"][0])
ACT_MAX = actuator_length(LIMITS["j2_boom_lift"][1])


def boom_angle(length: float) -> float:
    """Inverse of actuator_length: boom angle for an actuator length (law of cosines)."""
    a = math.hypot(PIVOT_X - ACT_LOW[0], PIVOT_Z - ACT_LOW[1])   # lower mount to pivot
    b = ACT_ON_BOOM
    cos_g = (a * a + b * b - length * length) / (2 * a * b)
    if not -1.0 <= cos_g <= 1.0:
        raise ValueError(f"actuator length {length:.3f} m is outside the linkage's range")
    gamma = math.acos(cos_g)                       # angle at the pivot between column and boom
    column_dir = math.atan2(ACT_LOW[1] - PIVOT_Z, ACT_LOW[0] - PIVOT_X)   # pointing down the column
    return column_dir + gamma


def actuator_pose(boom: float):
    """(pitch, length) of the actuator body for drawing it: angle from horizontal and length."""
    ax = PIVOT_X + ACT_ON_BOOM * math.cos(boom)
    az = PIVOT_Z + ACT_ON_BOOM * math.sin(boom)
    return math.atan2(az - ACT_LOW[1], ax - ACT_LOW[0]), math.hypot(ax - ACT_LOW[0], az - ACT_LOW[1])


def tool_position(q):
    """Fingertip position (x, y, z) for joint values q = [j1, j2, j3, j4, (j5)]."""
    q1, q2, q3 = q[0], q[1], q[2]
    pitch = q2 + q3                                # 0 = gripper pointing straight down
    r = PIVOT_X + BOOM * math.cos(q2) + TOOL * math.sin(pitch)
    z = PIVOT_Z + BOOM * math.sin(q2) - TOOL * math.cos(pitch)
    return (r * math.cos(q1), r * math.sin(q1), z)


def wrist_position(q):
    """Boom tip (wrist pitch axis) position."""
    r = PIVOT_X + BOOM * math.cos(q[1])
    return (r * math.cos(q[0]), r * math.sin(q[0]), PIVOT_Z + BOOM * math.sin(q[1]))


def level_wrist(q2: float) -> float:
    """Wrist pitch that keeps the gripper hanging straight down for boom angle q2."""
    return -q2


def within_limits(q) -> bool:
    return all(LIMITS[n][0] - 1e-9 <= v <= LIMITS[n][1] + 1e-9 for n, v in zip(JOINTS, q))


def inverse(x: float, y: float, z: float, pitch_hint: float = 0.0):
    """Joint values [j1, j2, j3, j4] putting the fingertips at (x, y, z).

    In the boom's vertical plane the boom (length BOOM) and the wrist
    (length TOOL) form a planar two-link arm, so there are up to two
    solutions; the one whose gripper pitch is closest to *pitch_hint*
    (0 = pointing down) is returned. None if out of reach or limits.
    """
    q1 = math.atan2(y, x)
    dr = math.hypot(x, y) - PIVOT_X
    dz = z - PIVOT_Z
    d2 = dr * dr + dz * dz
    c = (d2 - BOOM * BOOM - TOOL * TOOL) / (2 * BOOM * TOOL)
    if not -1.0 <= c <= 1.0:
        return None
    best = None
    for elbow in (math.acos(c), -math.acos(c)):          # angle of the wrist link relative to the boom
        q2 = math.atan2(dz, dr) - math.atan2(TOOL * math.sin(elbow), BOOM + TOOL * math.cos(elbow))
        pitch = q2 + elbow + math.pi / 2                  # wrist link direction is pitch - 90 deg
        pitch = math.atan2(math.sin(pitch), math.cos(pitch))
        q = [q1, q2, pitch - q2, 0.0]
        if within_limits(q) and (best is None or abs(pitch - pitch_hint) < abs(best[1] - pitch_hint)):
            best = (q, pitch)
    return None if best is None else best[0]


def jacobian(q, eps: float = 1e-6):
    """3 x 3 position Jacobian of the fingertips w.r.t. j1..j3 (numerical)."""
    base = tool_position(q)
    cols = []
    for i in range(3):
        dq = list(q)
        dq[i] += eps
        p = tool_position(dq)
        cols.append([(p[k] - base[k]) / eps for k in range(3)])
    return [[cols[c][r] for c in range(3)] for r in range(3)]
