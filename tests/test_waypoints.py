import math
from pathlib import Path

import pytest
import yaml

from lathe_arm_teach import arm_model as am
from lathe_arm_teach import cell

ROOT = Path(__file__).resolve().parents[1]
POINTS = yaml.safe_load((ROOT / "lathe_arm_teach/waypoints/example_load_cycle.yaml").read_text())["waypoints"]
BY_NAME = {p["name"]: p["joints"] for p in POINTS}

LEVEL = ("above_billet", "at_billet", "grip", "lift", "turn_to_spindle",
         "raise_to_clear", "over_machine", "into_chuck", "release", "clear_machine")


def test_five_values_each():
    assert len(POINTS) == 14
    for p in POINTS:
        assert len(p["joints"]) == 5, p["name"]


def test_within_limits():
    for p in POINTS:
        assert am.within_limits(p["joints"]), p["name"]


def test_starts_and_ends_at_home():
    assert POINTS[0]["joints"] == POINTS[-1]["joints"]


def test_gripper_open_and_closed_values():
    assert BY_NAME["above_billet"][4] == cell.OPEN
    assert BY_NAME["grip"][4] == cell.CLOSED
    assert BY_NAME["release"][4] == cell.OPEN


def test_wrist_stays_level_while_carrying():
    for name in LEVEL:
        q = BY_NAME[name]
        assert q[2] == pytest.approx(am.level_wrist(q[1]), abs=1e-3), name


def test_the_two_points_on_the_spindle_axis_are_nearly_level():
    """They sit on a straight line, not on the arm's level arc, so they are a
    little off. More than a couple of degrees and the billet would go in cocked."""
    for name in ("on_spindle_axis", "withdraw"):
        q = BY_NAME[name]
        assert abs(math.degrees(q[1] + q[2])) < 2.5, name


def test_fingertips_reach_the_billet():
    assert math.dist(am.tool_position(BY_NAME["at_billet"]), cell.PICK) < 0.01


def test_fingertips_reach_the_chuck():
    assert math.dist(am.tool_position(BY_NAME["into_chuck"]), cell.CHUCK) < 0.01


def test_billet_goes_in_along_the_spindle_axis():
    """The insertion has to run down the spindle axis, or the billet hits the jaws."""
    a = am.tool_position(BY_NAME["on_spindle_axis"])
    b = am.tool_position(BY_NAME["into_chuck"])
    travel = [b[i] - a[i] for i in range(3)]
    length = math.hypot(*travel)
    assert length == pytest.approx(cell.APPROACH, abs=0.005)
    # the billet must not rise or fall on the way in
    assert abs(travel[2]) < 0.002
    # and it must stay on the axis, not cut a chord across it
    off = sum(travel[i] * cell.AXIS_IN[i] for i in range(3))
    assert off == pytest.approx(length, abs=0.002)


def test_the_wrist_is_rolled_before_the_billet_goes_near_the_machine():
    """Picked lying radially, loaded lying along the spindle: a quarter turn."""
    assert BY_NAME["grip"][3] == pytest.approx(0.0, abs=1e-6)
    for name in ("turn_to_spindle", "over_machine", "on_spindle_axis", "into_chuck"):
        assert BY_NAME[name][3] == pytest.approx(cell.ROLL_FOR_CHUCK, abs=1e-3), name


def test_clearance_above_the_pick_and_the_machine():
    assert am.tool_position(BY_NAME["above_billet"])[2] > am.tool_position(BY_NAME["at_billet"])[2] + 0.05
    assert am.tool_position(BY_NAME["over_machine"])[2] > am.tool_position(BY_NAME["into_chuck"])[2] + 0.05


def test_the_swing_across_happens_at_one_height():
    """The machine's front guard is just under the carried billet. If the swing
    ramped up in height instead, the billet would clip it on the way in."""
    a = am.tool_position(BY_NAME["raise_to_clear"])
    b = am.tool_position(BY_NAME["over_machine"])
    assert a[2] == pytest.approx(b[2], abs=1e-4)
    assert a[2] == pytest.approx(cell.CHUCK_TIP_Z + cell.CLEAR, abs=1e-4)
    # only the base yaw changes between them
    for i in (1, 2, 3, 4):
        assert BY_NAME["raise_to_clear"][i] == pytest.approx(BY_NAME["over_machine"][i], abs=1e-4)


def test_the_carried_billet_clears_the_front_guard():
    for name in ("raise_to_clear", "over_machine", "clear_machine"):
        assert am.tool_position(BY_NAME[name])[2] > cell.GUARD_TOP, name
