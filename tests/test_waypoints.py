import math
from pathlib import Path

import pytest
import yaml

from lathe_arm_teach import arm_model as am

ROOT = Path(__file__).resolve().parents[1]
POINTS = yaml.safe_load((ROOT / "lathe_arm_teach/waypoints/example_load_cycle.yaml").read_text())["waypoints"]
BY_NAME = {p["name"]: p["joints"] for p in POINTS}

# grip points from lathe_cell.sdf
PART = (0.855, -0.599, 0.793)
CHUCK = (0.614, 0.877, 0.858)


def test_five_values_each():
    assert len(POINTS) == 11
    for p in POINTS:
        assert len(p["joints"]) == 5, p["name"]


def test_within_limits():
    for p in POINTS:
        assert am.within_limits(p["joints"]), p["name"]


def test_starts_and_ends_at_home():
    assert POINTS[0]["joints"] == POINTS[-1]["joints"]


def test_gripper_open_and_closed_values():
    assert BY_NAME["above_part"][4] == 0.030
    assert BY_NAME["grip"][4] == 0.0215
    assert BY_NAME["release"][4] == 0.030


def test_wrist_stays_level_at_the_pick_and_place_points():
    for name in ("above_part", "at_part", "lift", "above_chuck", "at_chuck"):
        q = BY_NAME[name]
        assert q[2] == pytest.approx(am.level_wrist(q[1]), abs=1e-3), name


def test_fingertips_reach_the_part():
    assert math.dist(am.tool_position(BY_NAME["at_part"]), PART) < 0.01


def test_fingertips_reach_the_chuck():
    assert math.dist(am.tool_position(BY_NAME["at_chuck"]), CHUCK) < 0.01


def test_above_points_are_higher_than_the_grip_points():
    assert am.tool_position(BY_NAME["above_part"])[2] > am.tool_position(BY_NAME["at_part"])[2] + 0.05
    assert am.tool_position(BY_NAME["above_chuck"])[2] > am.tool_position(BY_NAME["at_chuck"])[2] + 0.05
