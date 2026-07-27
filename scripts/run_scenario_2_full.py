"""场景2（2A+1B）手工分区流水线完整运行与输出。"""
from __future__ import annotations

import json
import subprocess
import os
import re
import sys
import time
from dataclasses import asdict, replace
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


def calculate_robot_time_accounting(
    event_log: list[dict], robot_ids: list[str]
) -> dict:
    """按互斥时间口径统计工作、运动、避让和等待时间。

    定义：
    - total_time = waiting_time + working_time
    - working_time = service_time + movement_time
    - movement_time = normal_movement_time + avoidance_time
    - avoidance_time 是执行避让路径时实际发生位置变化的时间
    """
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

    for rid, values in data.items():
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
            "total_time": "waiting_time + working_time; counted per robot until robot_finished",
            "working_time": "service_time + movement_time",
            "service_time": "time spent executing centrifuge operations, counted from work_tick events",
            "movement_time": "normal_movement_time + avoidance_time",
            "normal_movement_time": "position-changing move events outside yield paths",
            "avoidance_time": "position-changing move events while executing a yield path",
            "waiting_time": (
                "stationary non-service/non-movement time before robot_finished; "
                "plus post_completion_wait_time until global makespan; "
                "activation waits are listed separately in waiting_detail"
            ),
            "post_completion_wait_time": (
                "time after this robot returned/finished until the whole scenario ended"
            ),
        },
        "by_robot": data,
        "total": total,
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


def serialize_conflicts(conflicts) -> list[dict]:
    """把仿真层反馈转换为 JSON 可读格式。"""
    return [asdict(conflict) for conflict in conflicts]


def feedback_constraints_from_conflicts(
    conflicts,
    valid_operation_ids: set[str] | None = None,
) -> tuple[tuple[str, str, int], ...]:
    """提取可转成 CP-SAT precedence/delay 的反馈约束，并去重。"""
    unique: dict[tuple[str, str], tuple[str, str, int]] = {}
    for conflict in conflicts:
        suggested = conflict.suggested_precedence_constraint
        if suggested is None:
            continue
        before, after, delay = suggested
        if valid_operation_ids is not None and (
            before not in valid_operation_ids or after not in valid_operation_ids
        ):
            continue
        if before == after:
            continue
        key = (before, after)
        if key not in unique or delay > unique[key][2]:
            unique[key] = (before, after, max(1, delay))
    return tuple(unique.values())


def summarize_runtime_inefficiency(robot_time_accounting: dict) -> dict:
    """从仿真统计里提取闭环尚未转成 CP-SAT 约束的低效信号。"""
    by_robot = robot_time_accounting.get("by_robot", {})
    rows = []
    for rid, values in by_robot.items():
        waiting = int(values.get("waiting_time", 0))
        working = int(values.get("working_time", 0))
        total = int(values.get("accounting_total_time", 0))
        detail = values.get("waiting_detail", {})
        rows.append({
            "robot_id": rid,
            "waiting_time": waiting,
            "working_time": working,
            "waiting_ratio": round(waiting / total, 4) if total else 0,
            "precedence_wait_time": int(detail.get("precedence_wait_time", 0)),
            "conflict_wait_time": int(detail.get("conflict_wait_time", 0)),
            "retry_backoff_wait_time": int(detail.get("retry_backoff_wait_time", 0)),
            "safety_guard_wait_time": int(detail.get("safety_guard_wait_time", 0)),
            "yield_clearance_wait_time": int(detail.get("yield_clearance_wait_time", 0)),
        })
    rows.sort(key=lambda item: item["waiting_time"], reverse=True)
    return {
        "note": (
            "These are diagnostic signals for future soft penalties; "
            "only PlanningConflict.suggested_precedence_constraint is "
            "converted into a hard CP-SAT feedback constraint in this MVP."
        ),
        "by_waiting_time_desc": rows,
    }


def run_schedule_once(
    terrain,
    machines,
    operations,
    robots,
    schedule,
    started: float,
):
    """执行一次仿真并返回 engine/metrics/accounting 数据。"""
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
    robot_time_accounting = calculate_robot_time_accounting(
        engine.event_log, sorted(robots.keys())
    )
    return engine, metrics, machine_summary, robot_time_accounting, timing


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
    left_to_right_wave_experiments = {"test13", "test18"}
    enforce_column_blocks = experiment in {
        "test7", "test8", "test9", "test10", "test11", "test12", "test13",
        "test17", "test18", "test19", "test20",
    }
    enforce_disassembly_priority = experiment in {
        "test8", "test9", "test10", "test11", "test12", "test13", "test17",
        "test18", "test19", "test20",
    }
    enforce_b_follow_disassembly = experiment in {
        "test10", "test11", "test12", "test13", "test17", "test18", "test19",
        "test20",
    }
    minimize_initial_wait = experiment in {"test12", "test13", "test17", "test18", "test19", "test20"}
    allow_early_service_start = experiment in {"test12", "test13", "test17", "test18", "test19", "test20"}
    enforce_left_to_right_waves = experiment in left_to_right_wave_experiments
    cp_time_budget = 20 if experiment == "test17" else (120 if experiment == "test20" else 60)
    column_wave_order = ((2, 5), (8, 11), (14, 17), (20, 23)) if enforce_left_to_right_waves else ()
    preferred_inspection_order = (5, 2, 8, 11, 14, 17, 20, 23) if enforce_left_to_right_waves else ()
    preferred_a_column_order = (2, 5, 8, 11, 14, 17, 20, 23) if experiment == "test18" else ()
    install_order: tuple[int, ...] = ()
    if experiment == "test17":
        # test17 keeps the test12 CP-SAT constraint set and installation order.
        # The only accepted improvement is from the lighter runtime avoidance
        # behavior; probe run: test12 order -> sim 1094 (< original 1104).
        install_order = (11, 2, 5, 8, 14, 23, 20, 17)
    elif experiment == "test18":
        # User-specified intuitive pipeline:
        # A_1/A_2 process physical columns 1/2, 3/4, 5/6, 7/8 in pairs;
        # B inspects physical column 2 first, then 1,3,4,5,6,7,8.
        install_order = preferred_a_column_order
    elif experiment in {"test11", "test12", "test13", "test19", "test20"}:
        base_schedule = solve_assignment_schedule(
            terrain, machines, operations, robots,
            SolverConfig(
                max_time_seconds=cp_time_budget,
                allow_fallback=False,
                enforce_robot_column_blocks=enforce_column_blocks,
                column_blocks_by_operation_type=enforce_disassembly_priority,
                enforce_a_disassembly_priority=enforce_disassembly_priority,
                enforce_b_inspection_follows_disassembly_completion=True,
                disable_runtime_b_inspection_reorder=experiment == "test18",
                enforce_inspection_after_full_column_disassembly=enforce_left_to_right_waves,
                enforce_contiguous_bottom_up_inspection_chain=enforce_left_to_right_waves,
                column_wave_order=column_wave_order,
                enforce_disassembly_column_wave_order=enforce_left_to_right_waves,
                enforce_inspection_column_wave_order=enforce_left_to_right_waves,
                preferred_inspection_column_order=preferred_inspection_order,
                enforce_inspection_start_follows_preferred_order=enforce_left_to_right_waves,
                enforce_inspection_finish_column_before_next=enforce_left_to_right_waves,
                preferred_disassembly_column_order=preferred_a_column_order,
                enforce_alternating_disassembly_by_preferred_order=bool(preferred_a_column_order),
                enable_dynamic_avoidance_cost=experiment in {"test19", "test20"},
                minimize_initial_start_wait=minimize_initial_wait,
            ),
        )
        install_order = inspection_column_order(base_schedule, machines, operations)

    solver_config = SolverConfig(
            max_time_seconds=cp_time_budget,
            allow_fallback=False,
            enforce_robot_column_blocks=enforce_column_blocks,
            column_blocks_by_operation_type=enforce_disassembly_priority,
            enforce_a_disassembly_priority=enforce_disassembly_priority,
            enforce_b_inspection_follows_disassembly_completion=enforce_b_follow_disassembly,
            disable_runtime_b_inspection_reorder=experiment == "test18",
            enforce_inspection_after_full_column_disassembly=enforce_left_to_right_waves,
            enforce_contiguous_bottom_up_inspection_chain=enforce_left_to_right_waves,
            column_wave_order=column_wave_order,
            enforce_disassembly_column_wave_order=enforce_left_to_right_waves,
            enforce_inspection_column_wave_order=enforce_left_to_right_waves,
            enforce_install_column_wave_order=enforce_left_to_right_waves,
            preferred_inspection_column_order=preferred_inspection_order,
            enforce_inspection_start_follows_preferred_order=enforce_left_to_right_waves,
            enforce_inspection_finish_column_before_next=enforce_left_to_right_waves,
            preferred_disassembly_column_order=preferred_a_column_order,
            enforce_alternating_disassembly_by_preferred_order=bool(preferred_a_column_order),
            enable_dynamic_avoidance_cost=experiment in {"test19", "test20"},
            preferred_install_column_order=install_order,
            enforce_install_start_follows_preferred_order=bool(install_order),
            enforce_alternating_install_by_preferred_order=bool(install_order),
            minimize_initial_start_wait=minimize_initial_wait,
            allow_early_service_start=allow_early_service_start,
    )
    schedule = solve_assignment_schedule(
        terrain, machines, operations, robots, solver_config
    )

    closed_loop_feedback: dict = {
        "enabled": experiment == "test20",
        "strategy": (
            "simulate -> export structured conflicts -> add CP-SAT "
            "precedence/delay constraints -> re-solve"
        ),
        "iterations": [],
        "selected_iteration": 0,
        "accepted_constraints": [],
    }
    if experiment == "test20":
        max_feedback_iterations = 3
        feedback_constraints: tuple[tuple[str, str, int], ...] = ()
        best_tuple = None
        best_score = None
        current_schedule = schedule
        current_config = solver_config
        seen_constraints: set[tuple[str, str, int]] = set()

        for iteration in range(max_feedback_iterations + 1):
            (
                loop_engine,
                loop_metrics,
                loop_machine_summary,
                loop_robot_time_accounting,
                loop_timing,
            ) = run_schedule_once(
                terrain, machines, operations, robots, current_schedule, started
            )
            loop_conflicts = serialize_conflicts(loop_engine.planning_conflicts)
            new_constraints = feedback_constraints_from_conflicts(
                loop_engine.planning_conflicts,
                set(operations),
            )
            newly_accepted = [
                constraint for constraint in new_constraints
                if constraint not in seen_constraints
            ]
            for constraint in newly_accepted:
                seen_constraints.add(constraint)

            loop_record = {
                "iteration": iteration,
                "makespan": loop_metrics.makespan,
                "collisions": loop_metrics.collision_count,
                "constraint_violations": loop_metrics.constraint_violation_count,
                "precedence_violations": loop_metrics.precedence_violation_count,
                "replans": loop_metrics.number_of_replans,
                "planning_conflict_count": len(loop_engine.planning_conflicts),
                "new_feedback_constraints": [
                    list(constraint) for constraint in newly_accepted
                ],
                "runtime_inefficiency": summarize_runtime_inefficiency(
                    loop_robot_time_accounting
                ),
            }
            closed_loop_feedback["iterations"].append(loop_record)

            score = (
                loop_metrics.collision_count,
                loop_metrics.constraint_violation_count,
                loop_metrics.makespan,
                loop_metrics.number_of_replans,
            )
            if best_score is None or score < best_score:
                best_score = score
                best_tuple = (
                    iteration,
                    current_schedule,
                    loop_engine,
                    loop_metrics,
                    loop_machine_summary,
                    loop_robot_time_accounting,
                    loop_timing,
                    loop_conflicts,
                )

            if iteration == max_feedback_iterations or not newly_accepted:
                break

            feedback_constraints = tuple(
                dict.fromkeys(feedback_constraints + tuple(newly_accepted))
            )
            closed_loop_feedback["accepted_constraints"] = [
                list(constraint) for constraint in feedback_constraints
            ]
            current_config = replace(
                solver_config,
                additional_precedence_constraints=feedback_constraints,
            )
            try:
                current_schedule = solve_assignment_schedule(
                    terrain, machines, operations, robots, current_config
                )
            except Exception as exc:  # pragma: no cover - diagnostic path
                loop_record["repair_failed"] = str(exc)
                break

        assert best_tuple is not None
        (
            selected_iteration,
            schedule,
            engine,
            metrics,
            machine_summary,
            robot_time_accounting,
            timing,
            selected_conflicts,
        ) = best_tuple
        closed_loop_feedback["selected_iteration"] = selected_iteration
        closed_loop_feedback["selected_planning_conflict_count"] = len(
            selected_conflicts
        )
        (output / "planning_conflicts.json").write_text(
            json.dumps(selected_conflicts, indent=2, ensure_ascii=False)
        )
    else:
        (
            engine,
            metrics,
            machine_summary,
            robot_time_accounting,
            timing,
        ) = run_schedule_once(
            terrain, machines, operations, robots, schedule, started
        )
        (output / "planning_conflicts.json").write_text(
            json.dumps(
                serialize_conflicts(engine.planning_conflicts),
                indent=2,
                ensure_ascii=False,
            )
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
        "runtime_wait_yield_analysis": summarize_wait_and_yield(
            engine.event_log, sorted(robots.keys())
        ),
        "robot_time_accounting": robot_time_accounting,
        "closed_loop_feedback": closed_loop_feedback,
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
            "cp_time_budget": cp_time_budget,
            "enforce_robot_column_blocks": enforce_column_blocks,
            "column_blocks_by_operation_type": enforce_disassembly_priority,
            "enforce_a_disassembly_priority": enforce_disassembly_priority,
            "enforce_b_inspection_follows_disassembly_completion": enforce_b_follow_disassembly,
            "disable_runtime_b_inspection_reorder": experiment == "test18",
            "column_wave_order": [list(wave) for wave in column_wave_order],
            "enforce_disassembly_column_wave_order": enforce_left_to_right_waves,
            "enforce_inspection_column_wave_order": enforce_left_to_right_waves,
            "enforce_install_column_wave_order": enforce_left_to_right_waves,
            "preferred_inspection_column_order": list(preferred_inspection_order),
            "enforce_inspection_start_follows_preferred_order": enforce_left_to_right_waves,
            "enforce_inspection_finish_column_before_next": enforce_left_to_right_waves,
            "enforce_inspection_after_full_column_disassembly": enforce_left_to_right_waves,
            "enforce_contiguous_bottom_up_inspection_chain": enforce_left_to_right_waves,
            "preferred_disassembly_column_order": list(preferred_a_column_order),
            "enforce_alternating_disassembly_by_preferred_order": bool(preferred_a_column_order),
            "enable_dynamic_avoidance_cost": experiment in {"test19", "test20"},
            "closed_loop_feedback_enabled": experiment == "test20",
            "closed_loop_feedback_iterations": (
                len(closed_loop_feedback["iterations"])
                if experiment == "test20" else 0
            ),
            "additional_precedence_constraints": [
                list(constraint)
                for constraint in getattr(
                    schedule, "solver_objective", {}
                ).get("additional_precedence_constraints", [])
            ],
            "dynamic_avoidance_penalty": getattr(
                schedule, "solver_objective", {}
            ).get("dynamic_avoidance_penalty"),
            "dynamic_avoidance_time_buffer": getattr(
                schedule, "solver_objective", {}
            ).get("dynamic_avoidance_time_buffer"),
            "dynamic_avoidance_column_distance": getattr(
                schedule, "solver_objective", {}
            ).get("dynamic_avoidance_column_distance"),
            "dynamic_avoidance_weight": getattr(
                schedule, "solver_objective", {}
            ).get("dynamic_avoidance_weight"),
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
    (output / "closed_loop_feedback.json").write_text(
        json.dumps(closed_loop_feedback, indent=2, ensure_ascii=False)
    )
    (output / "robot_time_accounting.json").write_text(
        json.dumps(robot_time_accounting, indent=2, ensure_ascii=False)
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
