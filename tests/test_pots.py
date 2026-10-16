import math

import pytest

from lathe_arm_teach import arm_model as am
from lathe_arm_teach import pots


def test_channel_maps_raw_to_fraction():
    ch = pots.Channel(raw_min=100, raw_max=300)
    assert ch.fraction(100) == 0.0
    assert ch.fraction(200) == pytest.approx(0.5)
    assert ch.fraction(300) == 1.0
    assert ch.fraction(500) == 1.0       # clamped
    assert ch.fraction(0) == 0.0


def test_channel_invert():
    ch = pots.Channel(raw_min=0, raw_max=10, invert=True)
    assert ch.fraction(0) == 1.0
    assert ch.fraction(10) == 0.0
    assert ch.fraction(2.5) == pytest.approx(0.75)


def test_channel_zero_span():
    assert pots.Channel(raw_min=5, raw_max=5).fraction(7) == 0.0


def test_ends_of_travel():
    low = pots.fractions_to_joints([0, 0, 0, 0, 0])
    high = pots.fractions_to_joints([1, 1, 1, 1, 1])
    lim = am.LIMITS
    assert low[0] == pytest.approx(lim["j1_base_yaw"][0])
    assert high[0] == pytest.approx(lim["j1_base_yaw"][1])
    assert low[1] == pytest.approx(lim["j2_boom_lift"][0], abs=1e-6)
    assert high[1] == pytest.approx(lim["j2_boom_lift"][1], abs=1e-6)
    assert high[4] == pytest.approx(0.035)


def test_boom_knob_is_actuator_stroke():
    mid = pots.fractions_to_joints([0.5, 0.5, 0.5, 0.5, 0.5])
    expected = 0.5 * (am.ACT_MIN + am.ACT_MAX)
    assert am.actuator_length(mid[1]) == pytest.approx(expected, abs=1e-6)


def test_fraction_round_trip():
    for f in ([0.2, 0.3, 0.4, 0.6, 0.7], [0.9, 0.1, 0.5, 0.5, 0.0]):
        q = pots.fractions_to_joints(f)
        assert pots.joints_to_fractions(q) == pytest.approx(f, abs=1e-6)


def test_keep_level():
    for f2 in (0.1, 0.5, 0.9):
        q = pots.fractions_to_joints([0.5, f2, 0.9, 0.5, 0.5], keep_level=True)
        assert q[2] == pytest.approx(am.level_wrist(q[1]))


def test_filter_first_value_passes_straight_through():
    f = pots.PotFilter()
    assert f.update([0.2] * 5) == [0.2] * 5


def test_filter_smooths():
    f = pots.PotFilter(alpha=0.3)
    f.update([0.0] * 5)
    out = f.update([1.0] * 5)
    assert out == pytest.approx([0.3] * 5)


def test_deadband():
    f = pots.PotFilter(deadband=0.01)
    assert f.changed([0.5] * 5)                   # first one always goes out
    assert not f.changed([0.505] * 5)
    assert f.changed([0.52] * 5)
    assert not f.changed([0.525] * 5)             # compared with the last one sent


def test_ads1115():
    assert pots.ads1115_to_fraction(0) == 0.0
    assert pots.ads1115_to_fraction(13200) == pytest.approx(0.5)
    assert pots.ads1115_to_fraction(40000) == 1.0
    assert pots.ads1115_to_fraction(-30) == 0.0


def test_degrees_gripper_in_mm():
    out = pots.degrees([math.pi, 0, 0, 0, 0.0125])
    assert out[0] == pytest.approx(180.0)
    assert out[4] == pytest.approx(25.0)
