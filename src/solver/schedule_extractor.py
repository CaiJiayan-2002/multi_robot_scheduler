"""只沿 CP-SAT arc 解提取操作序列，不进行任务重排。"""
from __future__ import annotations

from ..domain.enums import ResultStatus
from ..domain.models import RobotSchedule, ScheduleResult, SchedulingProblem


def extract_cp_sat_schedule(problem, config, solver, status, artifacts) -> ScheduleResult:
    status_name = solver.StatusName(status)
    result = ScheduleResult(
        status=(ResultStatus.SUCCESS.value if status_name == "OPTIMAL" else ResultStatus.FEASIBLE.value),
        solver_backend="ortools_cp_sat",
        solver_mode="assignment_schedule",
        solver_status=status_name,
        makespan=solver.Value(artifacts.makespan),
        estimated_total_travel_time=solver.Value(artifacts.total_travel),
        column_switch_count=solver.Value(artifacts.column_switches),
        load_gap=solver.Value(artifacts.load_gap),
        solve_time_seconds=solver.WallTime(),
        best_objective_bound=solver.BestObjectiveBound(),
        fallback_used=False,
        fallback_reason=None,
        operation_sequence_source="cp_sat",
    )
    result.solver_objective = {
        "makespan": result.makespan,
        "estimated_total_travel_time": result.estimated_total_travel_time,
        "column_switch_count": result.column_switch_count,
        "load_gap": result.load_gap,
        "preference_penalty": solver.Value(artifacts.preference_penalty),
        "max_first_start": solver.Value(artifacts.max_first_start),
        "total_first_start": solver.Value(artifacts.total_first_start),
        "makespan_tolerance": config.makespan_tolerance,
        "enforce_robot_column_blocks": config.enforce_robot_column_blocks,
        "column_blocks_by_operation_type": config.column_blocks_by_operation_type,
        "enforce_a_disassembly_priority": config.enforce_a_disassembly_priority,
        "enforce_b_inspection_follows_disassembly_completion": (
            config.enforce_b_inspection_follows_disassembly_completion
        ),
        "disable_runtime_b_inspection_reorder": (
            config.disable_runtime_b_inspection_reorder
        ),
        "column_wave_order": [list(wave) for wave in config.column_wave_order],
        "enforce_disassembly_column_wave_order": (
            config.enforce_disassembly_column_wave_order
        ),
        "enforce_inspection_column_wave_order": (
            config.enforce_inspection_column_wave_order
        ),
        "enforce_install_column_wave_order": (
            config.enforce_install_column_wave_order
        ),
        "preferred_inspection_column_order": list(
            config.preferred_inspection_column_order
        ),
        "enforce_inspection_start_follows_preferred_order": (
            config.enforce_inspection_start_follows_preferred_order
        ),
        "enforce_inspection_finish_column_before_next": (
            config.enforce_inspection_finish_column_before_next
        ),
        "preferred_disassembly_column_order": list(
            config.preferred_disassembly_column_order
        ),
        "enforce_alternating_disassembly_by_preferred_order": (
            config.enforce_alternating_disassembly_by_preferred_order
        ),
        "enforce_same_b_robot_for_column_inspection": (
            config.enforce_same_b_robot_for_column_inspection
        ),
        "enforce_inspection_after_full_column_disassembly": (
            config.enforce_inspection_after_full_column_disassembly
        ),
        "enforce_contiguous_bottom_up_inspection_chain": (
            config.enforce_contiguous_bottom_up_inspection_chain
        ),
        "preferred_install_column_order": list(config.preferred_install_column_order),
        "enforce_install_start_follows_preferred_order": (
            config.enforce_install_start_follows_preferred_order
        ),
        "enforce_alternating_install_by_preferred_order": (
            config.enforce_alternating_install_by_preferred_order
        ),
        "minimize_initial_start_wait": config.minimize_initial_start_wait,
        "allow_early_service_start": config.allow_early_service_start,
        "random_seed": config.random_seed,
        "num_search_workers": config.num_search_workers,
    }
    result.objective = dict(result.solver_objective)

    for rid in problem.robots:
        successor = {}
        selected_arc_names = {}
        for (arc_rid, source, target), var in artifacts.arcs.items():
            if arc_rid == rid and solver.Value(var):
                successor[source] = target
                selected_arc_names[(source, target)] = var.Name()
        ordered = []
        current = successor.get("START", "END")
        seen = set()
        while current != "END":
            if current in seen or current not in problem.operations:
                raise RuntimeError(f"invalid CP-SAT successor chain for {rid}: {current}")
            seen.add(current)
            ordered.append(current)
            current = successor.get(current, "END")

        details = []
        previous = "START"
        for index, op_id in enumerate(ordered):
            op = problem.operations[op_id]
            next_id = successor.get(op_id, "END")
            travel = problem.travel_times[(rid, previous, op_id)]
            detail = {
                "operation_id": op_id,
                "machine_id": op.machine_id,
                "operation_type": op.operation_type.value,
                "assigned_robot_id": rid,
                "planned_start_time": solver.Value(artifacts.start[op_id]),
                "planned_end_time": solver.Value(artifacts.end[op_id]),
                "predecessor_operation_id": None if previous == "START" else previous,
                "successor_operation_id": None if next_id == "END" else next_id,
                "travel_time_from_predecessor": travel,
                "sequence_index": index,
                "arc_variable_name": selected_arc_names[(previous, op_id)],
            }
            details.append(detail)
            result.assignments.append(detail.copy())
            previous = op_id

        schedule = RobotSchedule(
            robot_id=rid,
            operations=[
                (d["operation_id"], d["planned_start_time"], d["planned_end_time"])
                for d in details
            ],
            ordered_operations=details,
            total_service_time=sum(problem.operations[o].duration for o in ordered),
            estimated_travel_time=(
                sum(d["travel_time_from_predecessor"] for d in details)
                + (problem.travel_times[(rid, ordered[-1], "END")] if ordered else 0)
            ),
            first_operation=ordered[0] if ordered else None,
            last_operation=ordered[-1] if ordered else None,
        )
        result.robot_schedules[rid] = schedule
    return result
