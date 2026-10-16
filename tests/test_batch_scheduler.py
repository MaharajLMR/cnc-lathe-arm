"""Tests for analysis/batch_scheduler.py, no ROS required."""

import sys
import random
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "analysis"))
from batch_scheduler import BatchScheduler, CYCLE_TIME_S, CHANGEOVER_S, CHANGEOVER_THRESHOLD_MM


def make_scheduler(n=10, seed=42):
    rng = random.Random(seed)
    sched = BatchScheduler()
    for i in range(n):
        sched.add_part(
            part_id=f"P{i+1:02d}",
            diameter_mm=rng.uniform(30, 80),
            material=rng.choice(["AL", "STEEL", "BRASS"]),
            priority=rng.randint(1, 3),
        )
    return sched


def test_all_parts_scheduled():
    sched = make_scheduler(10)
    for strategy in ["priority_fifo", "diameter_asc", "naive_fifo"]:
        ordered, _ = sched.schedule(strategy)
        assert len(ordered) == 10, f"Expected 10 parts, got {len(ordered)}"
        assert set(ordered) == {p.part_id for p in sched.parts}


def test_priority_fifo_high_priority_comes_first():
    sched = BatchScheduler()
    sched.add_part("A", 50, "AL", priority=3)
    sched.add_part("B", 50, "AL", priority=1)
    sched.add_part("C", 50, "AL", priority=2)
    ordered, _ = sched.schedule("priority_fifo")
    assert ordered[0] == "B", f"Highest priority (1) should be first, got {ordered[0]}"
    assert ordered[-1] == "A", f"Lowest priority (3) should be last, got {ordered[-1]}"


def test_diameter_asc_orders_by_diameter():
    sched = BatchScheduler()
    sched.add_part("big", 75, "AL", priority=1)
    sched.add_part("small", 30, "AL", priority=1)
    sched.add_part("mid", 55, "AL", priority=1)
    ordered, _ = sched.schedule("diameter_asc")
    assert ordered == ["small", "mid", "big"], f"Expected ascending diameter, got {ordered}"


def test_batch_time_includes_changeover():
    sched = BatchScheduler(cycle_time_s=5.0)
    sched.add_part("A", 30, "AL", priority=1)
    sched.add_part("B", 80, "AL", priority=1)  # diameter jump > 20 mm
    result = sched.simulate_batch("naive_fifo")
    expected_min = 2 * 5.0 + CHANGEOVER_S
    assert result["total_time_s"] >= expected_min - 1e-6, (
        f"Expected >= {expected_min:.2f}s, got {result['total_time_s']:.2f}s"
    )
    assert result["n_changeovers"] == 1


def test_no_changeover_when_diameters_close():
    sched = BatchScheduler(cycle_time_s=5.0)
    sched.add_part("A", 50, "AL", priority=1)
    sched.add_part("B", 55, "AL", priority=1)  # difference < 20 mm
    result = sched.simulate_batch("naive_fifo")
    assert result["n_changeovers"] == 0


def test_utilization_bounded():
    sched = make_scheduler(10)
    for strategy in ["priority_fifo", "diameter_asc", "naive_fifo"]:
        result = sched.simulate_batch(strategy)
        assert 0 < result["utilization_pct"] <= 100.0, (
            f"Utilisation {result['utilization_pct']}% out of range"
        )


def test_compare_strategies_returns_all_three():
    sched = make_scheduler(10)
    results = sched.compare_strategies()
    assert set(results.keys()) == {"priority_fifo", "diameter_asc", "naive_fifo"}


def test_diameter_asc_minimizes_changeovers():
    sched = BatchScheduler()
    # Parts with large diameter gaps, diameter_asc should sort them to minimize changeovers
    for d in [30, 80, 45, 75, 60]:
        sched.add_part(f"P{d}", float(d), "AL", priority=1)
    res_asc = sched.simulate_batch("diameter_asc")
    res_fifo = sched.simulate_batch("naive_fifo")
    assert res_asc["n_changeovers"] <= res_fifo["n_changeovers"], (
        "diameter_asc should have <= changeovers vs naive_fifo"
    )
