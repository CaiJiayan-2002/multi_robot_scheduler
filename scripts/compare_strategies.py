"""同一 1A1B 场景对比两个手工基线与完整 CP-SAT。"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT))

from src.domain.enums import RobotType
from src.domain.models import Cell, RobotSpec
from src.evaluation.metrics import MetricsCalculator
from src.map.fixed_map import FixedMap
from src.map.pose_graph import PoseGraph
from src.domain.models import Footprint
from src.simulation.engine import SimulationEngine
from src.solver.baselines import build_single_pair_baseline
from src.solver.config import SolverConfig
from src.solver.scheduler import solve_assignment_schedule
from src.solver.travel_time import build_operation_travel_times


def main():
    terrain, machines, operations = FixedMap().build()
    robots = {
        "A_1": RobotSpec("A_1", RobotType.A, Cell(1, 28)),
        "B_1": RobotSpec("B_1", RobotType.B, Cell(24, 28)),
    }
    strategies = {
        name: build_single_pair_baseline(machines, operations, robots, name)
        for name in ("row_major_baseline", "column_major_baseline")
    }
    graph = PoseGraph(terrain, Footprint.default_2x4())
    graph.build()
    static_travel = build_operation_travel_times(
        graph, Footprint.default_2x4(), operations, robots
    )
    for schedule in strategies.values():
        travel_total = 0
        switches = 0
        for rid, robot_schedule in schedule.robot_schedules.items():
            previous = "START"
            previous_x = None
            for op_id, _, _ in robot_schedule.operations:
                travel_total += static_travel[(rid, previous, op_id)]
                x = machines[operations[op_id].machine_id].cells[0].x
                if previous_x is not None and x != previous_x:
                    switches += 1
                previous, previous_x = op_id, x
            if previous != "START":
                travel_total += static_travel[(rid, previous, "END")]
        schedule.estimated_total_travel_time = travel_total
        schedule.column_switch_count = switches
    solver_started = time.perf_counter()
    strategies["cp_sat_assignment_schedule"] = solve_assignment_schedule(
        terrain, machines, operations, robots,
        SolverConfig(max_time_seconds=10, allow_fallback=False),
    )
    cp_solve_wall = time.perf_counter() - solver_started

    report = {}
    for name, schedule in strategies.items():
        started = time.perf_counter()
        engine = SimulationEngine()
        engine.setup(terrain, machines, operations, robots, schedule)
        engine.run(10000)
        path_wall = time.perf_counter() - started
        metrics = MetricsCalculator.compute(
            engine.event_log, engine.robots, engine.state_machine,
            engine.current_time, {"simulation": path_wall, "total_wall": path_wall},
        )
        report[name] = {
            "solver_backend": schedule.solver_backend,
            "sequence_source": schedule.operation_sequence_source,
            "fallback_used": schedule.fallback_used,
            "solver_status": schedule.solver_status,
            "makespan": metrics.makespan,
            "total_path_length": metrics.total_path_length,
            "estimated_travel_time": schedule.estimated_total_travel_time,
            "total_wait": metrics.total_wait,
            "column_switch_count": schedule.column_switch_count,
            "utilization": metrics.utilization_by_robot,
            "load_gap": schedule.load_gap,
            "solver_time_seconds": cp_solve_wall if name.startswith("cp_sat") else 0.0,
            "path_planning_time_seconds": path_wall,
            "collisions": metrics.collision_count,
            "completed_machines": engine.state_machine.summary().get("COMPLETED", 0),
        }
    output = PROJECT / "outputs" / "strategy_comparison.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
