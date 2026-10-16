"""Tests for analysis/fault_detector.py, no ROS required."""

import sys
import math
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "analysis"))
from fault_detector import FaultDetector, JOINT_VEL_LIMIT_RAD_S, SAFETY_ZONE
from fk_kinematics import JOINT_LIMITS

Q_MIN = np.array([lo for lo, _ in JOINT_LIMITS])
Q_MAX = np.array([hi for _, hi in JOINT_LIMITS])


@pytest.fixture
def detector():
    return FaultDetector()


def _flat_trajectory(q, n=20):
    """Constant joint config trajectory (static pose, ~zero velocity)."""
    configs = np.tile(q, (n, 1))
    times = np.linspace(0.0, 1.0, n)
    return times, configs


def test_clean_trajectory_no_limit_faults(detector):
    """A safe in-workspace config should not trigger E01."""
    q = np.array([0.0, 0.8, -1.5, 0.5, 0.0])
    times, configs = _flat_trajectory(q)
    faults = detector.check_trajectory(times, configs)
    e01 = [f for f in faults if f["fault_code"] == "E01"]
    assert e01 == [], f"Unexpected E01 faults: {e01}"


def test_joint_limit_fault_detected(detector):
    """A config outside joint limits must raise E01."""
    q = np.array([0.0, 2.5, 0.0, 0.0, 0.0])  # J2 > 1.9 rad, exceeds limit
    times, configs = _flat_trajectory(q)
    faults = detector.check_trajectory(times, configs)
    codes = [f["fault_code"] for f in faults]
    assert "E01" in codes, "E01 expected for joint beyond limit"


def test_velocity_fault_detected(detector):
    """A sudden position jump between samples creates large velocity → E02."""
    n = 10
    configs = np.zeros((n, 5))
    configs[5, 0] = 3.0   # 3 rad in ~0.1 s = 30 rad/s >> 2.0 rad/s limit × 1.5
    times = np.linspace(0.0, 1.0, n)
    faults = detector.check_trajectory(times, configs)
    codes = [f["fault_code"] for f in faults]
    assert "E02" in codes, "E02 expected for large velocity step"


def test_singularity_fault_at_home(detector):
    """Home pose (all zeros, arm fully extended) is singular → E04."""
    q = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    times, configs = _flat_trajectory(q, n=5)
    faults = detector.check_trajectory(times, configs)
    codes = [f["fault_code"] for f in faults]
    assert "E04" in codes, (
        "E04 expected at home pose (singular, arm fully extended)"
    )


def test_safety_zone_fault(detector):
    """An EE trajectory that passes through the operator zone must raise E05."""
    from fk_kinematics import ee_position
    # The safety zone is x:[-0.4,0.2], y:[-0.1,0.1], z:[0.5,1.2]
    # Arm folded up and back over the base: EE ≈ (-0.16, -0.03, 0.73), inside the zone
    q = np.array([0.2, -0.9, -1.6, -0.2, 0.1])
    pos = ee_position(q)
    sz = SAFETY_ZONE
    assert (sz["x_min"] <= pos[0] <= sz["x_max"] and
            sz["y_min"] <= pos[1] <= sz["y_max"] and
            sz["z_min"] <= pos[2] <= sz["z_max"]), f"test pose EE {pos} left the zone"
    times, configs = _flat_trajectory(q, n=5)
    faults = detector.check_trajectory(times, configs)
    codes = [f["fault_code"] for f in faults]
    assert "E05" in codes


def test_noise_simulation_perturbs_values(detector):
    configs = np.ones((10, 5)) * 0.5
    noisy = detector.simulate_noise(configs, noise_pct=1.0, seed=7)
    diff = np.abs(noisy - configs)
    assert diff.max() > 1e-6, "Noise simulation should change values"
    assert diff.max() < 0.1, "Noise should be small (<10%)"


def test_fault_free_rate_bounds(detector):
    q = np.array([0.0, 0.8, -1.5, 0.5, 0.0])
    times, configs = _flat_trajectory(q, n=20)
    faults = detector.check_trajectory(times, configs)
    ffr = detector.fault_free_rate(times, faults)
    assert 0.0 <= ffr <= 1.0


def test_no_false_positives_on_safe_trajectory(detector):
    """A trajectory well within limits and away from singularities must be clean."""
    n = 30
    # Interpolate between two safe configs
    q0 = np.array([0.0, 0.8, -1.5, 0.5, 0.0])
    q1 = np.array([0.5, 0.7, -1.3, 0.4, 0.3])
    configs = np.array([q0 + t * (q1 - q0) for t in np.linspace(0, 1, n)])
    times = np.linspace(0.0, 2.0, n)
    faults = detector.check_trajectory(times, configs)
    # Allow E03 (noise is random) and E04 (manipulability varies), but no E01/E02/E05
    hard_faults = [f for f in faults if f["fault_code"] in ("E01", "E02", "E05")]
    assert hard_faults == [], f"Unexpected hard faults: {hard_faults}"
