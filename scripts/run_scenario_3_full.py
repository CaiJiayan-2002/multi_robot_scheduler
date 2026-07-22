"""场景3（4A+2B）完整 CP-SAT 调度、仿真与输出。"""
from __future__ import annotations

import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT))

from src.domain.enums import RobotType
from src.domain.models import Cell, Footprint, RobotSpec
from src.domain.validation import FootprintValidator
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

    for values in wait_by_robot.values():
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


def calculate_robot_time_accounting(
    event_log: list[dict], robot_ids: list[str]
) -> dict:
    """按互斥时间口径统计工作、运动、避让和等待时间。"""
    move_re = re.compile(r"^([AB]_\d+): -> \(([-\d]+),([-\d]+)\) t=(\d+) (\S+)")
    rid_re = re.compile(r"^([AB]_\d+):")

    data = {
        rid: {
            "finish_time": 0,
            "service_time": 0,
            "normal_movement_time": 0,
            "avoidance_time": 0,
            "movement_time": 0,
            "working_time": 0,
            "waiting_time": 0,
            "waiting_detail": {
                "activation_wait_time": 0,
                "scheduled_path_wait_time": 0,
                "scheduled_dispatch_wait_time": 0,
                "precedence_wait_time": 0,
                "conflict_wait_time": 0,
                "retry_backoff_wait_time": 0,
                "safety_guard_wait_time": 0,
                "yield_clearance_wait_time": 0,
                "other_wait_time": 0,
                "residual_idle_wait_time": 0,
                "post_completion_wait_time": 0,
            },
            "accounting_total_time": 0,
            "equation_check": {},
        }
        for rid in robot_ids
    }
    yield_intervals: dict[str, list[tuple[int, int]]] = {rid: [] for rid in robot_ids}
    open_yield: dict[str, int] = {}
    last_event_t = 0

    for event in event_log:
        t = int(event.get("t", 0))
        last_event_t = max(last_event_t, t)
        event_type = event.get("type", "")
        message = event.get("message", "")
        rid_match = rid_re.match(message)
        rid = rid_match.group(1) if rid_match else ""
        if rid not in data:
            continue

        if event_type == "work_tick":
            data[rid]["service_time"] += 1
        elif event_type == "robot_finished":
            data[rid]["finish_time"] = max(data[rid]["finish_time"], t)
        elif event_type == "yield_planned":
            open_yield[rid] = t
        elif event_type == "yield_complete":
            start = open_yield.pop(rid, t)
            yield_intervals[rid].append((start, t))
        elif event_type == "move":
            move_match = move_re.match(message)
            action = move_match.group(5) if move_match else ""
            if action == "WAIT":
                detail = data[rid]["waiting_detail"]
                detail["scheduled_path_wait_time"] += 1
                detail["activation_wait_time"] += 1
            elif action not in ("", "START"):
                in_yield = any(start <= t <= end for start, end in yield_intervals[rid])
                if rid in open_yield:
                    in_yield = in_yield or open_yield[rid] <= t
                if in_yield:
                    data[rid]["avoidance_time"] += 1
                else:
                    data[rid]["normal_movement_time"] += 1
        elif event_type.startswith("wait_") and event_type.endswith("_tick"):
            reason = event_type.removeprefix("wait_").removesuffix("_tick")
            detail = data[rid]["waiting_detail"]
            if reason == "precedence":
                detail["precedence_wait_time"] += 1
                detail["activation_wait_time"] += 1
            elif reason == "scheduled":
                detail["scheduled_dispatch_wait_time"] += 1
                detail["activation_wait_time"] += 1
            elif reason == "conflict":
                detail["conflict_wait_time"] += 1
            elif reason == "retry_backoff":
                detail["retry_backoff_wait_time"] += 1
            elif reason == "safety_guard":
                detail["safety_guard_wait_time"] += 1
            elif reason == "yield_clearance":
                detail["yield_clearance_wait_time"] += 1
            else:
                detail["other_wait_time"] += 1

    for values in data.values():
        if values["finish_time"] <= 0:
            values["finish_time"] = last_event_t
        values["movement_time"] = (
            values["normal_movement_time"] + values["avoidance_time"]
        )
        values["working_time"] = values["service_time"] + values["movement_time"]
        observed_wait = sum(
            amount
            for key, amount in values["waiting_detail"].items()
            if key not in ("activation_wait_time", "post_completion_wait_time")
        )
        residual = max(0, values["finish_time"] - values["working_time"] - observed_wait)
        values["waiting_detail"]["residual_idle_wait_time"] = residual
        post_completion_wait = max(0, last_event_t - values["finish_time"])
        values["waiting_detail"]["post_completion_wait_time"] = post_completion_wait
        values["waiting_time"] = observed_wait + residual + post_completion_wait
        values["accounting_total_time"] = values["waiting_time"] + values["working_time"]
        values["global_makespan"] = last_event_t
        values["equation_check"] = {
            "total_equals_waiting_plus_working": (
                values["accounting_total_time"]
                == values["waiting_time"] + values["working_time"]
            ),
            "working_equals_service_plus_movement": (
                values["working_time"]
                == values["service_time"] + values["movement_time"]
            ),
            "movement_equals_normal_plus_avoidance": (
                values["movement_time"]
                == values["normal_movement_time"] + values["avoidance_time"]
            ),
            "total_equals_global_makespan": (
                values["accounting_total_time"] == last_event_t
            ),
        }

    total = {
        key: sum(values[key] for values in data.values())
        for key in (
            "service_time",
            "normal_movement_time",
            "avoidance_time",
            "movement_time",
            "working_time",
            "waiting_time",
            "accounting_total_time",
        )
    }
    total["waiting_detail"] = {
        key: sum(values["waiting_detail"][key] for values in data.values())
        for key in next(iter(data.values()))["waiting_detail"]
    } if data else {}

    return {
        "definitions": {
            "total_time": "waiting_time + working_time; counted per robot until global makespan",
            "working_time": "service_time + movement_time",
            "service_time": "time spent executing centrifuge operations",
            "movement_time": "normal_movement_time + avoidance_time",
            "normal_movement_time": "position-changing move events outside yield paths",
            "avoidance_time": "position-changing move events while executing a yield path",
            "waiting_time": "stationary non-service/non-movement time plus post-completion wait",
            "post_completion_wait_time": (
                "time after this robot returned/finished until the whole scenario ended"
            ),
        },
        "by_robot": data,
        "total": total,
    }


def audit_trajectory_collisions(event_log: list[dict], robot_ids: list[str]) -> dict:
    """对最终轨迹做 footprint 和 swept collision 复核。"""
    move_re = re.compile(r"^([AB]_\d+): -> \(([-\d]+),([-\d]+)\) t=(\d+) (\S+)")
    footprint = Footprint.default_2x4()
    raw: dict[int, dict[str, Cell]] = {}
    end_t = max((int(event.get("t", 0)) for event in event_log), default=0)
    for event in event_log:
        if event.get("type") != "move":
            continue
        match = move_re.match(event.get("message", ""))
        if not match:
            continue
        rid, x, y, t, _ = match.groups()
        raw.setdefault(int(t), {})[rid] = Cell(int(x), int(y))

    last: dict[str, Cell] = {}
    positions: dict[int, dict[str, Cell]] = {}
    for t in range(end_t + 1):
        if t in raw:
            last.update(raw[t])
        positions[t] = {rid: last[rid] for rid in robot_ids if rid in last}

    pose_collisions = []
    swept_conflicts = []
    for t in range(end_t + 1):
        ids = sorted(positions[t])
        for i, rid_a in enumerate(ids):
            cells_a = footprint.cells_at(positions[t][rid_a])
            for rid_b in ids[i + 1:]:
                overlap = cells_a & footprint.cells_at(positions[t][rid_b])
                if overlap:
                    pose_collisions.append({
                        "t": t,
                        "robots": [rid_a, rid_b],
                        "anchors": {
                            rid_a: [positions[t][rid_a].x, positions[t][rid_a].y],
                            rid_b: [positions[t][rid_b].x, positions[t][rid_b].y],
                        },
                        "overlap_cells": sorted([cell.x, cell.y] for cell in overlap),
                    })

    for t in range(1, end_t + 1):
        ids = sorted(set(positions[t - 1]) & set(positions[t]))
        sweeps = {
            rid: FootprintValidator.swept_cells(
                positions[t - 1][rid], positions[t][rid], footprint
            )
            for rid in ids
        }
        for i, rid_a in enumerate(ids):
            for rid_b in ids[i + 1:]:
                overlap = sweeps[rid_a] & sweeps[rid_b]
                if overlap:
                    swept_conflicts.append({
                        "t_interval": [t - 1, t],
                        "robots": [rid_a, rid_b],
                        "from_to": {
                            rid_a: [
                                [positions[t - 1][rid_a].x, positions[t - 1][rid_a].y],
                                [positions[t][rid_a].x, positions[t][rid_a].y],
                            ],
                            rid_b: [
                                [positions[t - 1][rid_b].x, positions[t - 1][rid_b].y],
                                [positions[t][rid_b].x, positions[t][rid_b].y],
                            ],
                        },
                        "overlap_cells": sorted([cell.x, cell.y] for cell in overlap),
                    })

    return {
        "pose_collision_count": len(pose_collisions),
        "swept_conflict_count": len(swept_conflicts),
        "collision_free": not pose_collisions and not swept_conflicts,
        "pose_collisions": pose_collisions[:100],
        "swept_conflicts": swept_conflicts[:100],
    }


def inspection_column_order(schedule, machines, operations) -> tuple[int, ...]:
    """按所有 B 机器人计划检测启动时间合并提取首次检测列顺序。"""
    candidates: list[tuple[int, int, str]] = []
    for rid, robot_schedule in schedule.robot_schedules.items():
        if not rid.startswith("B"):
            continue
        for detail in robot_schedule.ordered_operations:
            op_id = detail["operation_id"]
            op = operations[op_id]
            if op.operation_type.value != "INSPECT":
                continue
            x = machines[op.machine_id].cells[0].x
            planned_start = int(detail.get("planned_start_time", 0))
            candidates.append((planned_start, x, op_id))

    order: list[int] = []
    seen: set[int] = set()
    for _, x, _ in sorted(candidates):
        if x in seen:
            continue
        seen.add(x)
        order.append(x)
    return tuple(order)


def main() -> None:
    experiment = sys.argv[1] if len(sys.argv) > 1 else "test0"
    output = PROJECT / "outputs" / "scenario_3" / experiment
    output.mkdir(parents=True, exist_ok=True)

    started = time.perf_counter()
    terrain, machines, operations = FixedMap().build()
    footprint = Footprint.default_2x4()
    robots = {
        "A_1": RobotSpec("A_1", RobotType.A, Cell(1, 28), footprint),
        "A_2": RobotSpec("A_2", RobotType.A, Cell(5, 28), footprint),
        "A_3": RobotSpec("A_3", RobotType.A, Cell(9, 28), footprint),
        "A_4": RobotSpec("A_4", RobotType.A, Cell(13, 28), footprint),
        "B_1": RobotSpec("B_1", RobotType.B, Cell(17, 28), footprint),
        "B_2": RobotSpec("B_2", RobotType.B, Cell(21, 28), footprint),
    }

    base_config = dict(
        max_time_seconds=60,
        allow_fallback=False,
        enforce_robot_column_blocks=True,
        column_blocks_by_operation_type=True,
        enforce_a_disassembly_priority=True,
        enforce_b_inspection_follows_disassembly_completion=True,
        enforce_same_b_robot_for_column_inspection=True,
        enforce_inspection_after_full_column_disassembly=True,
        enforce_contiguous_bottom_up_inspection_chain=True,
        minimize_initial_start_wait=True,
    )
    print("[scenario_3] solving base CP-SAT schedule...", flush=True)
    base_schedule = solve_assignment_schedule(
        terrain, machines, operations, robots, SolverConfig(**base_config)
    )
    install_order = inspection_column_order(base_schedule, machines, operations)

    print(f"[scenario_3] solving final CP-SAT schedule, install_order={install_order}...", flush=True)
    schedule = solve_assignment_schedule(
        terrain,
        machines,
        operations,
        robots,
        SolverConfig(
            **base_config,
            preferred_install_column_order=install_order,
            enforce_install_start_follows_preferred_order=bool(install_order),
            enforce_alternating_install_by_preferred_order=bool(install_order),
            allow_early_service_start=True,
        ),
    )

    engine = SimulationEngine()
    engine.setup(terrain, machines, operations, robots, schedule)
    engine.progress_interval = int(os.environ.get("MRS_PROGRESS_INTERVAL", "0"))
    print("[scenario_3] running simulation...", flush=True)
    max_steps = int(os.environ.get("MRS_MAX_STEPS", "60000"))
    engine.run(max_steps=max_steps)
    timing = {
        "simulation": time.perf_counter() - started,
        "total_wall": time.perf_counter() - started,
    }
    metrics = MetricsCalculator.compute(
        engine.event_log, engine.robots, engine.state_machine,
        engine.current_time, timing,
    )
    machine_summary = engine.state_machine.summary()
    robot_ids = sorted(robots.keys())
    robot_time_accounting = calculate_robot_time_accounting(engine.event_log, robot_ids)
    collision_audit = audit_trajectory_collisions(engine.event_log, robot_ids)

    data = {
        "scenario": "4A2B",
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
            engine.event_log, robot_ids
        ),
        "robot_time_accounting": robot_time_accounting,
        "trajectory_collision_audit": {
            "pose_collision_count": collision_audit["pose_collision_count"],
            "swept_conflict_count": collision_audit["swept_conflict_count"],
            "collision_free": collision_audit["collision_free"],
        },
        "collisions_violations": {
            "collisions": metrics.collision_count,
            "pose_collisions_audited": collision_audit["pose_collision_count"],
            "swept_conflicts_audited": collision_audit["swept_conflict_count"],
            "constraint_violations": metrics.constraint_violation_count,
            "precedence_violations": metrics.precedence_violation_count,
        },
        "planning_quality": {
            "replans": metrics.number_of_replans,
            "solver_backend": schedule.solver_backend,
            "solver_mode": schedule.solver_mode,
            "solver_status": schedule.solver_status,
            "sequence_source": schedule.operation_sequence_source,
            "enforce_robot_column_blocks": True,
            "column_blocks_by_operation_type": True,
            "enforce_a_disassembly_priority": True,
            "enforce_b_inspection_follows_disassembly_completion": True,
            "enforce_same_b_robot_for_column_inspection": True,
            "enforce_inspection_after_full_column_disassembly": True,
            "enforce_contiguous_bottom_up_inspection_chain": True,
            "preferred_install_column_order": list(install_order),
            "enforce_install_start_follows_preferred_order": bool(install_order),
            "enforce_alternating_install_by_preferred_order": bool(install_order),
            "minimize_initial_start_wait": True,
            "allow_early_service_start": True,
        },
        "machine_completion": machine_summary,
        "timing": timing,
    }
    (output / "metrics.json").write_text(
        json.dumps(data, indent=2, ensure_ascii=False)
    )
    (output / "robot_time_accounting.json").write_text(
        json.dumps(robot_time_accounting, indent=2, ensure_ascii=False)
    )
    (output / "collision_audit.json").write_text(
        json.dumps(collision_audit, indent=2, ensure_ascii=False)
    )
    with (output / "event_log.jsonl").open("w") as file:
        for event in engine.event_log:
            file.write(json.dumps(event, ensure_ascii=False) + "\n")

    if os.environ.get("MRS_SKIP_RENDER") != "1":
        print("[scenario_3] rendering plots...", flush=True)
        render_python = os.environ.get("MRS_RENDER_PYTHON", "/opt/anaconda3/bin/python")
        if not Path(render_python).exists():
            render_python = sys.executable
        subprocess.run([
            render_python,
            str(PROJECT / "scripts" / "render_scenario_outputs.py"),
            experiment,
            "Scenario 3 (4A2B CP-SAT)",
            "scenario_3",
        ], check=True)

    completed_ops = sum(len(robot.completed_ops) for robot in engine.robots.values())
    print(json.dumps({
        "output": str(output),
        "makespan": engine.current_time,
        "completed_machines": machine_summary.get("COMPLETED", 0),
        "completed_operations": completed_ops,
        "collisions": metrics.collision_count,
        "pose_collisions_audited": collision_audit["pose_collision_count"],
        "swept_conflicts_audited": collision_audit["swept_conflict_count"],
        "violations": metrics.constraint_violation_count,
        "replans": metrics.number_of_replans,
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
