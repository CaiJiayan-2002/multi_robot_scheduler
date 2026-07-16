"""场景2（2A+1B）手工分区流水线完整运行与输出。"""
from __future__ import annotations

import json
import subprocess
import os
import sys
import time
from pathlib import Path

PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT))

from src.domain.enums import RobotType
from src.domain.models import Cell, Footprint, RobotSpec
from src.evaluation.metrics import MetricsCalculator
from src.map.fixed_map import FixedMap
from src.simulation.engine import SimulationEngine
from src.solver.config import SolverConfig
from src.solver.scheduler import solve_assignment_schedule


def summarize_wait_and_yield(event_log: list[dict], robot_ids: list[str]) -> dict:
    """按机器人拆分运行期等待与主动避让耗时。"""
    wait_by_robot = {
        rid: {
            "scheduled_path_wait": 0,
            "precedence_wait": 0,
            "conflict_wait": 0,
            "other_wait": 0,
            "yield_time": 0,
        }
        for rid in robot_ids
    }
    first_work_start = {rid: None for rid in robot_ids}
    open_yield: dict[str, int] = {}

    for event in event_log:
        t = int(event.get("t", 0))
        event_type = event.get("type", "")
        message = event.get("message", "")
        rid = message.split(":", 1)[0] if ":" in message else ""
        if rid not in wait_by_robot:
            continue

        if event_type == "work_start" and first_work_start[rid] is None:
            first_work_start[rid] = t
        elif event_type == "move" and " WAIT" in message:
            wait_by_robot[rid]["scheduled_path_wait"] += 1
        elif event_type in {"wait_precedence", "precedence_cleared"}:
            wait_by_robot[rid]["precedence_wait"] += 1
        elif event_type == "wait_conflict":
            wait_by_robot[rid]["conflict_wait"] += 1
        elif event_type.startswith("wait"):
            wait_by_robot[rid]["other_wait"] += 1
        elif event_type == "yield_planned":
            open_yield[rid] = t
        elif event_type == "yield_complete":
            start = open_yield.pop(rid, t)
            wait_by_robot[rid]["yield_time"] += max(0, t - start)

    for rid, values in wait_by_robot.items():
        values["total_wait_without_yield"] = (
            values["scheduled_path_wait"]
            + values["precedence_wait"]
            + values["conflict_wait"]
            + values["other_wait"]
        )
        values["total_wait_with_yield"] = (
            values["total_wait_without_yield"] + values["yield_time"]
        )

    return {
        "first_work_start": first_work_start,
        "by_robot": wait_by_robot,
        "total": {
            key: sum(values[key] for values in wait_by_robot.values())
            for key in (
                "scheduled_path_wait",
                "precedence_wait",
                "conflict_wait",
                "other_wait",
                "yield_time",
                "total_wait_without_yield",
                "total_wait_with_yield",
            )
        },
    }


def inspection_column_order(schedule, machines, operations) -> tuple[int, ...]:
    """从 CP-SAT 结果中提取 B 机器人首次检测各列的顺序。"""
    order: list[int] = []
    seen: set[int] = set()
    for rid, robot_schedule in schedule.robot_schedules.items():
        if not rid.startswith("B"):
            continue
        for detail in robot_schedule.ordered_operations:
            op_id = detail["operation_id"]
            op = operations[op_id]
            if op.operation_type.value != "INSPECT":
                continue
            x = machines[op.machine_id].cells[0].x
            if x not in seen:
                seen.add(x)
                order.append(x)
    return tuple(order)


def main() -> None:
    experiment = sys.argv[1] if len(sys.argv) > 1 else "260703_test11"
    output = PROJECT / "outputs" / "scenario_2" / experiment
    output.mkdir(parents=True, exist_ok=True)

    started = time.perf_counter()
    terrain, machines, operations = FixedMap().build()
    footprint = Footprint.default_2x4()
    robots = {
        "A_1": RobotSpec("A_1", RobotType.A, Cell(1, 28), footprint),
        "A_2": RobotSpec("A_2", RobotType.A, Cell(12, 28), footprint),
        "B_1": RobotSpec("B_1", RobotType.B, Cell(24, 28), footprint),
    }
    enforce_column_blocks = experiment in {"test7", "test8", "test9", "test10", "test11", "test12"}
    enforce_disassembly_priority = experiment in {"test8", "test9", "test10", "test11", "test12"}
    enforce_b_follow_disassembly = experiment in {"test10", "test11", "test12"}
    minimize_initial_wait = experiment in {"test12"}
    allow_early_service_start = experiment in {"test12"}
    install_order: tuple[int, ...] = ()
    if experiment in {"test11", "test12"}:
        base_schedule = solve_assignment_schedule(
            terrain, machines, operations, robots,
            SolverConfig(
                max_time_seconds=60,
                allow_fallback=False,
                enforce_robot_column_blocks=enforce_column_blocks,
                column_blocks_by_operation_type=enforce_disassembly_priority,
                enforce_a_disassembly_priority=enforce_disassembly_priority,
                enforce_b_inspection_follows_disassembly_completion=True,
                minimize_initial_start_wait=minimize_initial_wait,
            ),
        )
        install_order = inspection_column_order(base_schedule, machines, operations)

    schedule = solve_assignment_schedule(
        terrain, machines, operations, robots,
        SolverConfig(
            max_time_seconds=60,
            allow_fallback=False,
            enforce_robot_column_blocks=enforce_column_blocks,
            column_blocks_by_operation_type=enforce_disassembly_priority,
            enforce_a_disassembly_priority=enforce_disassembly_priority,
            enforce_b_inspection_follows_disassembly_completion=enforce_b_follow_disassembly,
            preferred_install_column_order=install_order,
            enforce_install_start_follows_preferred_order=bool(install_order),
            enforce_alternating_install_by_preferred_order=bool(install_order),
            minimize_initial_start_wait=minimize_initial_wait,
            allow_early_service_start=allow_early_service_start,
        ),
    )

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
        "runtime_wait_yield_analysis": summarize_wait_and_yield(
            engine.event_log, sorted(robots.keys())
        ),
        "collisions_violations": {
            "collisions": metrics.collision_count,
            "constraint_violations": metrics.constraint_violation_count,
            "precedence_violations": metrics.precedence_violation_count,
        },
        "planning_quality": {
            "replans": metrics.number_of_replans,
            "solver_backend": schedule.solver_backend,
            "solver_mode": schedule.solver_mode,
            "solver_status": schedule.solver_status,
            "sequence_source": schedule.operation_sequence_source,
            "enforce_robot_column_blocks": enforce_column_blocks,
            "column_blocks_by_operation_type": enforce_disassembly_priority,
            "enforce_a_disassembly_priority": enforce_disassembly_priority,
            "enforce_b_inspection_follows_disassembly_completion": enforce_b_follow_disassembly,
            "preferred_install_column_order": list(install_order),
            "enforce_install_start_follows_preferred_order": bool(install_order),
            "enforce_alternating_install_by_preferred_order": bool(install_order),
            "minimize_initial_start_wait": minimize_initial_wait,
            "allow_early_service_start": allow_early_service_start,
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
    render_python = os.environ.get("MRS_RENDER_PYTHON", "/opt/anaconda3/bin/python")
    if not Path(render_python).exists():
        render_python = sys.executable
    subprocess.run([
        render_python,
        str(PROJECT / "scripts" / "render_scenario_outputs.py"),
        experiment,
        "Scenario 2 (2A1B CP-SAT)",
        "scenario_2",
    ], check=True)

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
