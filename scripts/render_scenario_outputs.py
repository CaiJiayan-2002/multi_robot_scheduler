"""在独立进程渲染图表，隔离 OR-Tools/protobuf 与 Matplotlib native 库。"""
from __future__ import annotations

import json
import sys
from pathlib import Path

PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT))

from src.evaluation.plots import GanttChart, TrajectoryPlot
from src.map.fixed_map import FixedMap


def main():
    experiment = sys.argv[1]
    label = sys.argv[2] if len(sys.argv) > 2 else experiment
    scenario_dir = sys.argv[3] if len(sys.argv) > 3 else "scenario_1"
    output = PROJECT / "outputs" / scenario_dir / experiment
    events = [json.loads(line) for line in (output / "event_log.jsonl").read_text().splitlines()]
    terrain, machines, _ = FixedMap().build()
    makespan = max((event["t"] for event in events), default=0)
    gantt = GanttChart.build_gantt_data(events)
    GanttChart.save_gantt_png(
        gantt, str(output / "gantt.png"), title=f"{label} — Makespan={makespan}"
    )
    trajectories = TrajectoryPlot.build_trajectory_data(events)
    TrajectoryPlot.save_trajectory_json(trajectories, str(output / "trajectories.json"))
    TrajectoryPlot.save_trajectory_png(
        trajectories, terrain, machines, str(output / "trajectories.png"),
        title=f"{label} — {makespan} steps",
    )


if __name__ == "__main__":
    main()
