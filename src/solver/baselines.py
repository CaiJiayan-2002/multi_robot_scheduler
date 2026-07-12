"""显式手工基线；正式 CP-SAT 模式禁止导入本模块。"""
from __future__ import annotations

from ..domain.enums import ResultStatus, RobotType
from ..domain.models import RobotSchedule, ScheduleResult


def build_single_pair_baseline(machines, operations, robots, strategy: str):
    if strategy not in {"row_major_baseline", "column_major_baseline"}:
        raise ValueError(strategy)
    a_id = next(rid for rid, r in robots.items() if r.robot_type == RobotType.A)
    b_id = next(rid for rid, r in robots.items() if r.robot_type == RobotType.B)
    if strategy == "row_major_baseline":
        machine_ids = sorted(
            machines, key=lambda mid: (machines[mid].row, machines[mid].cells[0].x)
        )
    else:
        machine_ids = sorted(
            machines, key=lambda mid: (machines[mid].cells[0].x, machines[mid].row)
        )
    a_ops = [f"{mid}_D" for mid in machine_ids] + [f"{mid}_R" for mid in machine_ids]
    b_ops = [f"{mid}_I" for mid in machine_ids]
    result = ScheduleResult(
        status=ResultStatus.FEASIBLE.value,
        solver_backend="manual_baseline",
        solver_mode="baseline",
        solver_status="FEASIBLE",
        fallback_used=True,
        fallback_reason="explicit baseline requested",
        operation_sequence_source=strategy,
    )
    result.robot_schedules[a_id] = RobotSchedule(
        a_id, [(op_id, -1, -1) for op_id in a_ops]
    )
    result.robot_schedules[b_id] = RobotSchedule(
        b_id, [(op_id, -1, -1) for op_id in b_ops]
    )
    return result
