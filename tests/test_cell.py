"""The world file has the cell geometry written into it as literal numbers.

cell.py is the source of those numbers, so check the two still agree. If this
fails, run tools/make_waypoints.py and copy the poses it prints into
lathe_cell.sdf.
"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from lathe_arm_teach import cell

ROOT = Path(__file__).resolve().parents[1]
WORLD = ET.parse(ROOT / "lathe_arm_bringup/worlds/lathe_cell.sdf").getroot().find("world")


def model(name):
    m = WORLD.find(f"model[@name='{name}']")
    assert m is not None, f"no model named {name} in lathe_cell.sdf"
    return m


def pose_of(elem):
    return [float(v) for v in elem.find("pose").text.split()]


def test_tray_is_where_the_arm_reaches():
    x, y, _, _, _, yaw = pose_of(model("job_tray"))
    assert (x, y) == pytest.approx(cell.TRAY_ORIGIN[:2], abs=1e-3)
    assert yaw == pytest.approx(cell.TRAY_MODEL_YAW, abs=1e-3)


def test_lathe_is_where_the_arm_reaches():
    x, y, _, _, _, yaw = pose_of(model("lathe"))
    assert (x, y) == pytest.approx(cell.LATHE_ORIGIN[:2], abs=1e-3)
    assert yaw == pytest.approx(cell.LATHE_MODEL_YAW, abs=1e-3)


def test_chuck_sits_on_the_lathe():
    assert pose_of(model("chuck"))[:3] == pytest.approx(pose_of(model("lathe"))[:3], abs=1e-6)
    assert pose_of(model("chuck"))[5] == pytest.approx(pose_of(model("lathe"))[5], abs=1e-6)


def test_billet_starts_in_the_middle_pocket():
    x, y, z, _, pitch, yaw = pose_of(model("part"))
    assert (x, y, z) == pytest.approx(cell.BILLET, abs=1e-3)
    # laid on its side, pointing out along the tray's x
    assert pitch == pytest.approx(math.pi / 2, abs=1e-3)
    assert yaw == pytest.approx(cell.TRAY_YAW, abs=1e-3)


def test_spindle_is_horizontal_not_vertical():
    """The whole point of the rebuild: a lathe chuck faces sideways."""
    for name in ("spindle_nose", "quill"):
        v = model("lathe").find(f"link/visual[@name='{name}']")
        assert v is not None, name
        _, _, z, _, pitch, _ = [float(p) for p in v.find("pose").text.split()]
        # a cylinder is modelled along z, so a quarter turn about y lays it flat
        assert pitch == pytest.approx(math.pi / 2, abs=1e-3), name
        assert z == pytest.approx(cell.SPINDLE_Z, abs=1e-3), name


def test_chuck_body_is_on_the_spindle_line():
    body = model("chuck").find("link/visual[@name='body']")
    _, _, z, _, pitch, _ = [float(p) for p in body.find("pose").text.split()]
    assert z == pytest.approx(cell.SPINDLE_Z, abs=1e-3)
    assert pitch == pytest.approx(math.pi / 2, abs=1e-3)


def test_tray_holds_one_billet_per_pocket():
    billets = model("job_tray").findall("link/visual[@name]")
    names = [v.get("name") for v in billets if v.get("name").startswith("billet_")]
    # four dummies on the tray plus the live one that gets loaded
    assert len(names) == cell.POCKETS - 1


def test_chuck_can_take_the_billet_off_the_gripper():
    plugin = model("chuck").find("plugin")
    assert plugin is not None and "DetachableJoint" in plugin.get("name")
    assert plugin.find("child_model").text == "part"
    assert plugin.find("attach_topic").text == "/chuck/attach"


def test_the_loading_side_stays_under_the_arm():
    """The arm swings over the front guard rather than through a door, so the
    guard rail has to stay below the height the billet is carried at."""
    rail = model("lathe").find("link/visual[@name='guard_rail']")
    _, _, z = [float(v) for v in rail.find("pose").text.split()[:3]]
    thickness = float(rail.find("geometry/box/size").text.split()[2])
    assert z + thickness / 2 <= cell.GUARD_TOP + 1e-6


def test_the_machine_looks_like_a_lathe():
    """Cheap guard against the model quietly losing the parts that make it read
    as a CNC lathe rather than a grey box."""
    names = {v.get("name") for v in model("lathe").findall("link/visual")}
    for part in ("slant_bed", "headstock", "spindle_nose", "turret", "tailstock",
                 "quill", "window", "panel_screen", "light_green", "conveyor",
                 "door_panel", "guard_rail"):
        assert part in names, part


def test_every_camera_has_its_own_topic():
    topics = [s.find("topic").text
              for m in WORLD.findall("model")
              for s in m.findall("link/sensor")
              if s.get("type") == "camera" and s.find("topic") is not None]
    assert sorted(topics) == ["/cell/closeup", "/cell/image", "/cell/machine", "/cell/pick"]
