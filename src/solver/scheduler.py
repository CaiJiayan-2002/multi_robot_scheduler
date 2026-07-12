"""正式求解入口：默认完整 CP-SAT，禁止静默 fallback。"""
from __future__ import annotations

from dataclasses import replace
from ..domain.models import Footprint, Machine, Operation, RobotSpec, SchedulingProblem, ScheduleResult
from ..map.pose_graph import PoseGraph
from .config import SolverConfig
from .cp_sat_model import CpSatScheduler
from .travel_time import build_operation_travel_times


class SchedulingFailure(RuntimeError):
    pass


def solve_assignment_schedule(
    terrain,
    machines: dict[str, Machine],
    operations: dict[str, Operation],
    robots: dict[str, RobotSpec],
    config: SolverConfig | None = None,
) -> ScheduleResult:
    config = config or SolverConfig()
    if config.solver_mode != "assignment_schedule":
        raise ValueError("formal entrypoint requires solver_mode='assignment_schedule'")
    footprint = Footprint.default_2x4()
    graph = PoseGraph(terrain, footprint)
    graph.build()
    travel_times = build_operation_travel_times(graph, footprint, operations, robots)
    problem = SchedulingProblem(machines, operations, robots, travel_times)
    result = CpSatScheduler(config).solve(
        problem, config.max_time_seconds, config.solver_mode
    )
    if result.status not in ("success", "feasible"):
        if not config.allow_fallback:
            raise SchedulingFailure(result.fallback_reason or result.solver_status)
        # fallback 只能被显式开启，且结果必须明确标记来源。
        from .fallback import manual_assign
        fallback = manual_assign(problem)
        fallback.fallback_used = True
        fallback.fallback_reason = result.fallback_reason
        fallback.solver_backend = "manual_baseline"
        fallback.solver_mode = "baseline"
        fallback.solver_status = "FALLBACK"
        fallback.operation_sequence_source = "baseline"
        return fallback
    return result


def repair_schedule_from_conflicts(
    terrain,
    machines,
    operations,
    robots,
    conflicts,
    config: SolverConfig | None = None,
) -> ScheduleResult:
    """把路径层建议的 precedence/delay 反馈给 CP-SAT 后重新求解。"""
    base = config or SolverConfig()
    repairs = tuple(
        conflict.suggested_precedence_constraint
        for conflict in conflicts
        if conflict.suggested_precedence_constraint is not None
    )
    merged = tuple(dict.fromkeys(
        base.additional_precedence_constraints + repairs
    ))
    return solve_assignment_schedule(
        terrain, machines, operations, robots,
        replace(base, additional_precedence_constraints=merged),
    )
