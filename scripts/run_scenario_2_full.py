"""场景2（2A+1B）手工分区流水线完整运行与输出。"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT))

from src.domain.enums import RobotType
from src.domain.models import Cell, Footprint, RobotSpec
from src.evaluation.metrics import MetricsCalculator
from src.evaluation.plots import GanttChart, TrajectoryPlot
from src.map.fixed_map import FixedMap
from src.simulation.engine import SimulationEngine
from src.solver.fallback import manual_assign_scenario_2


def main() -> None:
    experiment = sys.argv[1] if len(sys.argv) > 1 else "260703_test11"
    output = PROJECT / "outputs" / "scenario_1" / experiment
    output.mkdir(parents=True, exist_ok=True)

    started = time.perf_counter()
    terrain, machines, operations = FixedMap().build()
    footprint = Footprint.default_2x4()
    robots = {
        "A_1": RobotSpec("A_1", RobotType.A, Cell(1, 28), footprint),
        "A_2": RobotSpec("A_2", RobotType.A, Cell(12, 28), footprint),
        "B_1": RobotSpec("B_1", RobotType.B, Cell(24, 28), footprint),
    }
    schedule = manual_assign_scenario_2(machines, operations, robots)

    engine = SimulationEngine()
    engine.setup(terrain, machines, operations, robots, schedule)
    engine.run(max_steps=60000)
    timing = {
        "simulation": time.perf_counter() - started,
        "total_wall": time.perf_counter() - started,
    }
    metrics = MetricsCalculator.compute(
        engine.event_log, engine.robots, engine.state_machine,
        engine.current_time, timing,
    )
    machine_summary = engine.state_machine.summary()

    gantt = GanttChart.build_gantt_data(engine.event_log)
    GanttChart.save_gantt_png(
        gantt, str(output / "gantt.png"),
        title=f"Scenario 2 (2A1B) — Makespan={engine.current_time}",
    )
    trajectories = TrajectoryPlot.build_trajectory_data(engine.event_log)
    TrajectoryPlot.save_trajectory_json(
        trajectories, str(output / "trajectories.json")
    )
    TrajectoryPlot.save_trajectory_png(
        trajectories, terrain, machines, str(output / "trajectories.png"),
        title=f"Scenario 2 (2A1B) — {engine.current_time} steps",
    )

    data = {
        "scenario": "2A1B",
        "makespan": metrics.makespan,
        "path_length": {
            "total": metrics.total_path_length,
            "by_robot": metrics.path_by_robot,
            "by_type": metrics.path_by_type,
        },
        "wait_times": {
            "total": metrics.total_wait,
            "by_robot": metrics.wait_by_robot,
        },
        "collisions_violations": {
            "collisions": metrics.collision_count,
            "constraint_violations": metrics.constraint_violation_count,
            "precedence_violations": metrics.precedence_violation_count,
        },
        "planning_quality": {
            "replans": metrics.number_of_replans,
            "method": schedule.objective.get("method"),
            "column_groups": schedule.objective.get("column_groups"),
        },
        "machine_completion": machine_summary,
        "timing": timing,
    }
    (output / "metrics.json").write_text(
        json.dumps(data, indent=2, ensure_ascii=False)
    )
    with (output / "event_log.jsonl").open("w") as file:
        for event in engine.event_log:
            file.write(json.dumps(event, ensure_ascii=False) + "\n")

    completed_ops = sum(len(robot.completed_ops) for robot in engine.robots.values())
    print(json.dumps({
        "output": str(output),
        "makespan": engine.current_time,
        "completed_machines": machine_summary.get("COMPLETED", 0),
        "completed_operations": completed_ops,
        "collisions": metrics.collision_count,
        "violations": metrics.constraint_violation_count,
        "replans": metrics.number_of_replans,
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
