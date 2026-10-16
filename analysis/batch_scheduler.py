"""
Multi-part batch scheduler for the CNC lathe loading arm.

Models automated batch runs of N workparts with:
  - Three scheduling strategies: priority_fifo, diameter_asc, naive_fifo
  - Gripper changeover penalty when diameter changes > 20 mm
  - Gantt chart visualisation

Usage:
  python3 analysis/batch_scheduler.py
"""

import json
import random
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

CYCLE_TIME_S = 5.55          # seconds per part (from cycle_time_estimator.py)
CHANGEOVER_S = 0.50          # gripper changeover if diameter change > 20 mm
CHANGEOVER_THRESHOLD_MM = 20.0

MATERIAL_COLORS = {"AL": "#4C9BE8", "STEEL": "#C0C0C0", "BRASS": "#D4A017"}


class WorkPart:
    def __init__(self, part_id: str, diameter_mm: float, material: str, priority: int):
        self.part_id = part_id
        self.diameter_mm = diameter_mm
        self.material = material
        self.priority = priority   # 1=high, 2=medium, 3=low

    def __repr__(self):
        return (f"WorkPart({self.part_id}, ⌀{self.diameter_mm:.0f}mm, "
                f"{self.material}, prio={self.priority})")


class BatchScheduler:
    def __init__(self, cycle_time_s: float = CYCLE_TIME_S):
        self.cycle_time_s = cycle_time_s
        self.parts: list[WorkPart] = []

    def add_part(self, part_id: str, diameter_mm: float, material: str, priority: int):
        self.parts.append(WorkPart(part_id, diameter_mm, material, priority))

    # Scheduling strategies ─────────────────────────────────────────────── #
    def _ordered_ids(self, strategy: str) -> list[str]:
        if strategy == "priority_fifo":
            # Primary: priority ascending (1=high first); secondary: insertion order
            return [p.part_id for p in sorted(self.parts, key=lambda p: p.priority)]
        elif strategy == "diameter_asc":
            return [p.part_id for p in sorted(self.parts, key=lambda p: p.diameter_mm)]
        elif strategy == "naive_fifo":
            return [p.part_id for p in self.parts]
        else:
            raise ValueError(f"Unknown strategy: {strategy}")

    def schedule(self, strategy: str = "priority_fifo"):
        ordered = self._ordered_ids(strategy)
        parts_map = {p.part_id: p for p in self.parts}
        t = 0.0
        for i, pid in enumerate(ordered):
            t += self.cycle_time_s
            if i > 0:
                prev = parts_map[ordered[i - 1]]
                curr = parts_map[pid]
                if abs(curr.diameter_mm - prev.diameter_mm) > CHANGEOVER_THRESHOLD_MM:
                    t += CHANGEOVER_S
        return ordered, t

    # Simulation ───────────────────────────────────────────────────────── #
    def simulate_batch(self, strategy: str):
        parts_map = {p.part_id: p for p in self.parts}
        ordered, _ = self.schedule(strategy)
        records = []
        t = 0.0
        for i, pid in enumerate(ordered):
            p = parts_map[pid]
            changeover = 0.0
            if i > 0:
                prev = parts_map[ordered[i - 1]]
                if abs(p.diameter_mm - prev.diameter_mm) > CHANGEOVER_THRESHOLD_MM:
                    changeover = CHANGEOVER_S
            start = t + changeover
            end = start + self.cycle_time_s
            records.append({
                "part_id": pid,
                "material": p.material,
                "diameter_mm": float(p.diameter_mm),
                "priority": p.priority,
                "changeover_s": changeover,
                "start_s": start,
                "end_s": end,
            })
            t = end
        total = records[-1]["end_s"] if records else 0.0
        n_changeovers = sum(1 for r in records if r["changeover_s"] > 0)
        utilization = (len(records) * self.cycle_time_s) / total if total > 0 else 1.0
        return {
            "strategy": strategy,
            "records": records,
            "total_time_s": float(round(total, 3)),
            "n_changeovers": n_changeovers,
            "utilization_pct": float(round(utilization * 100, 1)),
            "mean_wait_s": float(round(total / len(records), 3)) if records else 0.0,
        }

    # Compare strategies ───────────────────────────────────────────────── #
    def compare_strategies(self):
        strategies = ["priority_fifo", "diameter_asc", "naive_fifo"]
        return {s: self.simulate_batch(s) for s in strategies}


# Gantt chart ───────────────────────────────────────────────────────────── #
def plot_gantt(sim_result: dict, out_path: Path):
    records = sim_result["records"]
    fig, ax = plt.subplots(figsize=(14, max(4, len(records) * 0.45 + 1)))

    for i, r in enumerate(records):
        color = MATERIAL_COLORS.get(r["material"], "#888888")
        width = r["end_s"] - r["start_s"]
        ax.barh(i, width, left=r["start_s"], color=color, edgecolor="white", height=0.7)
        if r["changeover_s"] > 0:
            ax.barh(i, r["changeover_s"],
                    left=r["start_s"] - r["changeover_s"],
                    color="#FF6B6B", edgecolor="white", height=0.7, alpha=0.7)
        label = f"{r['part_id']} ⌀{r['diameter_mm']:.0f} P{r['priority']}"
        ax.text(r["start_s"] + width / 2, i, label,
                ha="center", va="center", fontsize=7, color="black")

    ax.set_yticks(range(len(records)))
    ax.set_yticklabels([r["part_id"] for r in records], fontsize=8)
    ax.set_xlabel("Time (s)")
    strat = sim_result["strategy"].replace("_", " ").title()
    ax.set_title(f"Batch Schedule, {strat}  |  "
                 f"Total: {sim_result['total_time_s']:.1f}s  |  "
                 f"Utilisation: {sim_result['utilization_pct']:.0f}%")

    patches = [mpatches.Patch(color=c, label=m) for m, c in MATERIAL_COLORS.items()]
    patches.append(mpatches.Patch(color="#FF6B6B", alpha=0.7, label="Changeover"))
    ax.legend(handles=patches, loc="upper right", fontsize=8)
    ax.grid(axis="x", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    print(f"Saved {out_path}")


# Main ──────────────────────────────────────────────────────────────────── #
def main():
    rng = random.Random(42)
    materials = ["AL", "STEEL", "BRASS"]

    scheduler = BatchScheduler()
    for i in range(10):
        scheduler.add_part(
            part_id=f"P{i+1:02d}",
            diameter_mm=rng.uniform(30, 80),
            material=rng.choice(materials),
            priority=rng.randint(1, 3),
        )

    results = scheduler.compare_strategies()

    # Best strategy by total time
    best_key = min(results, key=lambda k: results[k]["total_time_s"])
    best = results[best_key]

    out_dir = Path(__file__).parent / "results"
    out_dir.mkdir(exist_ok=True)

    plot_gantt(best, out_dir / "batch_schedule.png")

    # Save comparison JSON
    comparison = {}
    for strat, res in results.items():
        comparison[strat] = {
            "total_time_s": res["total_time_s"],
            "n_changeovers": res["n_changeovers"],
            "utilization_pct": res["utilization_pct"],
            "mean_wait_s": res["mean_wait_s"],
        }
    out_json = out_dir / "batch_comparison.json"
    with open(out_json, "w") as f:
        json.dump(comparison, f, indent=2)
    print(f"Saved {out_json}")

    # Print markdown table
    print("\n=== Batch Scheduling Strategy Comparison (10 parts, seed=42) ===")
    print(f"| Strategy        | Total time (s) | Changeovers | Utilisation |")
    print(f"|---|---|---|---|")
    for strat, res in results.items():
        print(f"| {strat:15s} | {res['total_time_s']:14.2f} | "
              f"{res['n_changeovers']:11d} | {res['utilization_pct']:10.0f}% |")
    print(f"\nBest: {best_key} ({best['total_time_s']:.2f} s)")


if __name__ == "__main__":
    main()
