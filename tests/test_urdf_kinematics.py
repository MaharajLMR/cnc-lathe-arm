"""
Tests for URDF structure and FK consistency, no ROS required.

Parses lathe_arm.urdf.xacro (as plain XML after stripping xacro macros)
and verifies joint structure, limits, and ee_link existence.
Also cross-checks FK at home position against fk_kinematics.py.
"""

import math
import sys
import os
import re
from pathlib import Path
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).parent.parent / "analysis"))
import fk_kinematics as fk


# helpers ──────────────────────────────────────────────────────────────── #
URDF_PATH = Path(__file__).parent.parent / "lathe_arm_description" / "urdf" / "lathe_arm.urdf.xacro"


def _resolve_xacro(text: str) -> str:
    """Replace ${expr} with evaluated float values for numeric property refs."""
    props = {}
    for m in re.finditer(r'<xacro:property name="(\w+)"\s+value="([^"]+)"', text):
        try:
            props[m.group(1)] = float(m.group(2))
        except ValueError:
            pass

    def replace_expr(match):
        expr = match.group(1)
        try:
            local = {"math": math, **props}
            return str(eval(expr, {"__builtins__": {}}, local))
        except Exception:
            return match.group(0)

    return re.sub(r'\$\{([^}]+)\}', replace_expr, text)


def _load_urdf() -> ET.Element:
    raw = URDF_PATH.read_text()
    resolved = _resolve_xacro(raw)
    # Strip xacro namespace prefix so ET can parse as plain XML
    resolved = resolved.replace("xacro:", "xacro_")
    return ET.fromstring(resolved)


# tests ─────────────────────────────────────────────────────────────────── #
def test_urdf_file_exists():
    assert URDF_PATH.exists(), f"URDF not found: {URDF_PATH}"


def test_five_revolute_joints():
    root = _load_urdf()
    revolute = [j for j in root.findall("joint") if j.get("type") == "revolute"]
    assert len(revolute) == 5, f"Expected 5 revolute joints, found {len(revolute)}"


def test_joint_names():
    root = _load_urdf()
    names = {j.get("name") for j in root.findall("joint") if j.get("type") == "revolute"}
    expected = {
        "joint_1_base_shoulder",
        "joint_2_shoulder_upper",
        "joint_3_upper_elbow",
        "joint_4_elbow_wrist",
        "joint_5_wrist_roll",
    }
    assert names == expected, f"Joint name mismatch. Found: {names}"


def test_joint_limits_match_spec():
    """Verify each joint's lower/upper limits match the industrial spec."""
    root = _load_urdf()
    spec = {
        "joint_1_base_shoulder":  (-1.5708,  1.5708),
        "joint_2_shoulder_upper": (-1.0,     1.9),
        "joint_3_upper_elbow":    (-2.1,     0.2),
        "joint_4_elbow_wrist":    (-1.5708,  1.5708),
        "joint_5_wrist_roll":     (-3.14159, 3.14159),
    }
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name not in spec:
            continue
        limit_el = joint.find("limit")
        assert limit_el is not None, f"{name}: no <limit> element"
        lower = float(limit_el.get("lower"))
        upper = float(limit_el.get("upper"))
        exp_lo, exp_hi = spec[name]
        assert abs(lower - exp_lo) < 0.01, f"{name} lower: got {lower}, expected {exp_lo}"
        assert abs(upper - exp_hi) < 0.01, f"{name} upper: got {upper}, expected {exp_hi}"


def test_ee_link_exists():
    root = _load_urdf()
    link_names = {l.get("name") for l in root.findall("link")}
    assert "ee_link" in link_names, f"ee_link not found. Links: {link_names}"


def test_ee_fixed_joint_connects_to_ee_link():
    root = _load_urdf()
    fixed_joints = [j for j in root.findall("joint") if j.get("type") == "fixed"]
    ee_joints = [j for j in fixed_joints
                 if j.find("child") is not None and j.find("child").get("link") == "ee_link"]
    assert len(ee_joints) == 1, "Expected exactly one fixed joint with child=ee_link"


def test_home_position_fk():
    """FK at all-zero joints: EE should be at (0.805, 0, 0.210) ± 0.005 m."""
    pos = fk.ee_position([0, 0, 0, 0, 0])
    # x = L2+L3+L4+L5 = 0.350+0.300+0.090+0.065 = 0.805
    # z = base_offset(0.030) + L1(0.180) = 0.210
    assert abs(pos[0] - 0.805) < 0.005, f"Home EE x={pos[0]:.4f}, expected 0.805"
    assert abs(pos[1] - 0.000) < 1e-9,  f"Home EE y={pos[1]:.4f}, expected 0"
    assert abs(pos[2] - 0.210) < 0.005, f"Home EE z={pos[2]:.4f}, expected 0.210"


def test_reach_within_spec():
    """Maximum reach at home = L2+L3+L4+L5+L1 vector length ≤ 0.985 m."""
    pos = fk.ee_position([0, 0, 0, 0, 0])
    reach = (pos[0]**2 + pos[1]**2 + pos[2]**2) ** 0.5
    assert reach <= 0.985 + 0.01, f"Reach {reach:.4f} m exceeds spec 0.985 m"


def test_joint_limits_in_fk_module():
    """fk_kinematics.py JOINT_LIMITS matches the updated industrial spec."""
    assert abs(fk.JOINT_LIMITS[1][0] - (-1.0))  < 0.01, "J2 lower mismatch"
    assert abs(fk.JOINT_LIMITS[1][1] -  1.9)    < 0.01, "J2 upper mismatch"
    assert abs(fk.JOINT_LIMITS[2][0] - (-2.1))  < 0.01, "J3 lower mismatch"
    assert abs(fk.JOINT_LIMITS[2][1] -  0.2)    < 0.01, "J3 upper mismatch"


def test_ros2_control_block_present():
    """URDF must contain the ros2_control hardware block for Gazebo."""
    raw = URDF_PATH.read_text()
    assert "gz_ros2_control/GazeboSimSystem" in raw, \
        "Missing ros2_control hardware plugin"
    assert "command_interface" in raw, \
        "Missing command_interface in ros2_control block"
