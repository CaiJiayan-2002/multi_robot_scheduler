"""完整 CP-SAT 场景验证；求解进程不加载绘图库。"""
from __future__ import annotations

import json
import sys
from pathlib import Path

PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT))

from src.domain.enums import RobotType
from src.domain.models import Cell, RobotSpec
from src.map.fixed_map import FixedMap
from src.simulation.engine import SimulationEngine
from src.solver.config import SolverConfig
from src.solver.scheduler import solve_assignment_schedule


SCENARIOS = {
    "1A1B": {
        "A_1": (RobotType.A, Cell(1, 28)),
        "B_1": (RobotType.B, Cell(24, 28)),
    },
    "2A1B": {
        "A_1": (RobotType.A, Cell(1, 28)),
        "A_2": (RobotType.A, Cell(12, 28)),
        "B_1": (RobotType.B, Cell(24, 28)),
    },
    "4A2B": {
        "A_1": (RobotType.A, Cell(1, 28)),
        "A_2": (RobotType.A, Cell(5, 28)),
        "A_3": (RobotType.A, Cell(9, 28)),
        "A_4": (RobotType.A, Cell(13, 28)),
        "B_1": (RobotType.B, Cell(17, 28)),
        "B_2": (RobotType.B, Cell(21, 28)),
    },
}


def serialize_schedule(result):
    return {
        "solver_backend": result.solver_backend,
        "solver_mode": result.solver_mode,
        "solver_status": result.solver_status,
        "solver_objective": result.solver_objective,
        "makespan": result.makespan,
        "estimated_total_travel_time": result.estimated_total_travel_time,
        "column_switch_count": result.column_switch_count,
        "load_gap": result.load_gap,
        "best_objective_bound": result.best_objective_bound,
        "solve_time_seconds": result.solve_time_seconds,
        "fallback_used": result.fallback_used,
        "fallback_reason": result.fallback_reason,
        "operation_sequence_source": result.operation_sequence_source,
        "robots": {
            rid: {
                "robot_id": rid,
                "ordered_operations": schedule.ordered_operations,
                "total_service_time": schedule.total_service_time,
                "estimated_travel_time": schedule.estimated_travel_time,
                "first_operation": schedule.first_operation,
                "last_operation": schedule.last_operation,
            }
            for rid, schedule in result.robot_schedules.items()
        },
    }


def main():
    terrain, machines, operations = FixedMap().build()
    output = PROJECT / "outputs" / "cp_sat_validation"
    output.mkdir(parents=True, exist_ok=True)
    report = {}
    for name, specs in SCENARIOS.items():
        robots = {
            rid: RobotSpec(rid, robot_type, start)
            for rid, (robot_type, start) in specs.items()
        }
        result = solve_assignment_schedule(
            terrain, machines, operations, robots,
            SolverConfig(max_time_seconds=15, allow_fallback=False),
        )
        scenario = serialize_schedule(result)
        engine = SimulationEngine()
        engine.setup(terrain, machines, operations, robots, result)
        engine.run(10000)
        scenario["path_validation"] = {
            "completed_operations": sum(
                len(runtime.completed_ops) for runtime in engine.robots.values()
            ),
            "completed_machines": engine.state_machine.summary().get("COMPLETED", 0),
            "actual_makespan": engine.current_time,
            "collisions": len(engine.get_events_by_type("collision")),
            "constraint_violations": len(engine.get_events_by_type("constraint_violation")),
            "planning_conflicts": [
                conflict.__dict__ for conflict in engine.planning_conflicts
            ],
        }
        report[name] = scenario
        (output / f"{name}.json").write_text(
            json.dumps(scenario, indent=2, ensure_ascii=False)
        )
    (output / "summary.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False)
    )
    print(json.dumps({name: {
        "status": item["solver_status"],
        "fallback": item["fallback_used"],
        "makespan": item["makespan"],
        "travel": item["estimated_total_travel_time"],
        "path_validation": item.get("path_validation"),
    } for name, item in report.items()}, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
