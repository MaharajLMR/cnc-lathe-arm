"""
Unit tests for teach mode and replay — no ROS required.

Tests:
  - Waypoint YAML round-trip (save then load)
  - Example load-cycle YAML is valid
  - Waypoint structure validation
"""

import yaml
import os
import sys
import tempfile
from pathlib import Path

# Allow importing without installing
sys.path.insert(0, str(Path(__file__).parent.parent / "lathe_arm_teach"))


def test_example_waypoints_valid():
    """The example_load_cycle.yaml has valid structure."""
    path = Path(__file__).parent.parent / \
           "lathe_arm_teach" / "waypoints" / "example_load_cycle.yaml"
    assert path.exists(), f"Waypoints file not found: {path}"

    with open(path) as f:
        data = yaml.safe_load(f)

    assert "waypoints" in data
    wps = data["waypoints"]
    assert len(wps) >= 4, f"Expected >= 4 waypoints, got {len(wps)}"

    for wp in wps:
        assert "name" in wp, f"Waypoint missing 'name': {wp}"
        assert "joints" in wp, f"Waypoint '{wp['name']}' missing 'joints'"
        assert len(wp["joints"]) == 5, \
            f"Waypoint '{wp['name']}': expected 5 joints, got {len(wp['joints'])}"


def test_waypoint_round_trip():
    """Save and load waypoints preserves all values."""
    waypoints = [
        {"name": "home",      "joints": [0.0, 0.0,  0.0, 0.0, 0.0]},
        {"name": "pick",      "joints": [0.5, -0.3, 0.8, -0.5, 0.0]},
        {"name": "place",     "joints": [1.2, -0.2, 0.6, -0.4, 0.0]},
    ]
    data = {"waypoints": waypoints}

    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        yaml.dump(data, f)
        tmp_path = f.name

    try:
        with open(tmp_path) as f:
            loaded = yaml.safe_load(f)

        assert len(loaded["waypoints"]) == len(waypoints)
        for orig, loaded_wp in zip(waypoints, loaded["waypoints"]):
            assert orig["name"] == loaded_wp["name"]
            for a, b in zip(orig["joints"], loaded_wp["joints"]):
                assert abs(a - b) < 1e-9, f"Joint value mismatch: {a} vs {b}"
    finally:
        os.unlink(tmp_path)


def test_all_waypoints_have_5_joints():
    """All waypoints in example_load_cycle.yaml have exactly 5 joint values."""
    path = Path(__file__).parent.parent / \
           "lathe_arm_teach" / "waypoints" / "example_load_cycle.yaml"
    with open(path) as f:
        data = yaml.safe_load(f)
    for wp in data["waypoints"]:
        assert len(wp["joints"]) == 5, \
            f"'{wp['name']}' has {len(wp['joints'])} joints, expected 5"


def test_joint_angles_within_limits():
    """All example waypoint angles are within the URDF limits."""
    # URDF limits (radians): j1=[-1.57,1.57], j2=[-1.57,1.57],
    #                        j3=[-2.09,2.09], j4=[-1.57,1.57], j5=[-3.14,3.14]
    limits = [
        (-1.571, 1.571),
        (-1.571, 1.571),
        (-2.094, 2.094),
        (-1.571, 1.571),
        (-3.14159, 3.14159),
    ]
    path = Path(__file__).parent.parent / \
           "lathe_arm_teach" / "waypoints" / "example_load_cycle.yaml"
    with open(path) as f:
        data = yaml.safe_load(f)
    for wp in data["waypoints"]:
        for i, (angle, (lo, hi)) in enumerate(zip(wp["joints"], limits)):
            assert lo <= angle <= hi, \
                f"'{wp['name']}' joint {i+1}={angle} outside [{lo}, {hi}]"
