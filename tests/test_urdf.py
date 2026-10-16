import math
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from lathe_arm_teach import arm_model as am

ROOT = Path(__file__).resolve().parents[1]
XACRO = ROOT / "lathe_arm_description/urdf/lathe_arm.urdf.xacro"

pytestmark = pytest.mark.skipif(shutil.which("xacro") is None, reason="xacro not installed")


@pytest.fixture(scope="module")
def robot():
    out = subprocess.run(["xacro", str(XACRO), "controllers:=/dev/null"], capture_output=True, text=True, check=True)
    root = ET.fromstring(out.stdout)
    return {j.get("name"): j for j in root.findall("joint")}


def rot(rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_rot(axis, angle):
    a = np.array(axis, float)
    a /= np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def floats(text, n=3):
    return [float(v) for v in (text or " ".join(["0"] * n)).split()]


def transform(joint, value=0.0):
    """4x4 transform parent -> child for a joint at *value*."""
    org = joint.find("origin")
    xyz = floats(org.get("xyz") if org is not None else None)
    rpy = floats(org.get("rpy") if org is not None else None)
    t = np.eye(4)
    t[:3, :3] = rot(rpy)
    t[:3, 3] = xyz
    m = np.eye(4)
    kind = joint.get("type")
    if kind == "revolute":
        m[:3, :3] = axis_rot(floats(joint.find("axis").get("xyz")), value)
    elif kind == "prismatic":
        m[:3, 3] = np.array(floats(joint.find("axis").get("xyz"))) * value
    return t @ m


def chain_to(robot, link):
    """Joints from the root down to *link*."""
    by_child = {j.find("child").get("link"): j for j in robot.values()}
    chain = []
    while link in by_child:
        chain.append(by_child[link])
        link = by_child[link].find("parent").get("link")
    return chain[::-1]


def link_pose(robot, link, values):
    t = np.eye(4)
    for j in chain_to(robot, link):
        t = t @ transform(j, values.get(j.get("name"), 0.0))
    return t


def test_tool_link_matches_arm_model(robot):
    for q in ([0.0, 0.1745, -0.1745, 0.0], [-0.61, -0.42, 0.42, 0.0], [0.96, -0.35, 0.35, 0.0], [1.2, 0.5, -0.2, 0.3]):
        values = dict(zip(am.JOINTS, q))
        tip = link_pose(robot, "tool_link", values)[:3, 3]
        assert tip == pytest.approx(am.tool_position(q), abs=1e-6)


def test_joint_limits_match_arm_model(robot):
    for name, (lo, hi) in am.LIMITS.items():
        lim = robot[name].find("limit")
        assert float(lim.get("lower")) == pytest.approx(lo, abs=1e-6)
        assert float(lim.get("upper")) == pytest.approx(hi, abs=1e-6)


def test_actuator_rod_meets_the_boom(robot):
    pytest.importorskip("rclpy")                    # linkage_node imports ROS
    from lathe_arm_teach.linkage_node import ROD_LENGTH

    for boom in (-0.5, 0.0, 0.4, 0.78):
        pitch, length = am.actuator_pose(boom)
        values = {"j2_boom_lift": boom, "act_pitch": pitch, "act_extend": length - ROD_LENGTH}
        rod_tip = link_pose(robot, "act_rod", values)[:3, 3] + link_pose(robot, "act_rod", values)[:3, 0] * ROD_LENGTH
        attach = link_pose(robot, "boom_link", values) @ np.array([am.ACT_ON_BOOM, 0, 0, 1])
        assert rod_tip == pytest.approx(attach[:3], abs=1e-6)
