"""手动任务分配 v4.0

在 CP-SAT (ortools) 不可用时的备选方案。
场景1(1A1B): A_1 做所有 DISASSEMBLE+INSTALL, B_1 做所有 INSPECT。
按 D->I->R 的前序关系排列操作顺序。
"""

from __future__ import annotations

from ..domain.enums import OperationType, RobotType, ResultStatus
from ..domain.models import (
    Machine, Operation, RobotSpec, RobotSchedule, ScheduleResult, SchedulingProblem,
)


def _machine_sort_key(machines: dict[str, Machine], mid: str) -> tuple[int, int]:
    """按机器位置自然排序（先 row 后 x）。"""
    m = machines[mid]
    return (m.row, m.cells[0].x)


def manual_assign_scenario_1(
    machines: dict[str, Machine],
    operations: dict[str, Operation],
    robots: dict[str, RobotSpec],
) -> ScheduleResult:
    """场景1手动分配: A_1 做所有 D 和 R, B_1 做所有 I。

    按 D->I->R 的前序关系排列每个机器的操作。
    同一台离心机的三个操作必须按顺序执行: D -> I -> R。
    B 的 INSPECT 必须等 A 的 DISASSEMBLE 完成（前序依赖）。

    操作顺序: 按机器行序排列，每台机器依次处理。
    先 DISASSEMBLE 全部（A），然后 INSPECT 全部（B），最后 INSTALL 全部（A），
    确保前序关系。

    Args:
        machines: 离心机字典
        operations: 操作字典
        robots: 机器人规格字典

    Returns:
        ScheduleResult 包含各机器人的操作序列
    """
    # 识别 A 和 B 机器人
    robot_a = None
    robot_b = None
    for rid, rspec in robots.items():
        if rspec.robot_type == RobotType.A:
            robot_a = rid
        elif rspec.robot_type == RobotType.B:
            robot_b = rid

    if robot_a is None:
        return ScheduleResult(
            status=ResultStatus.INVALID_INPUT.value,
            objective={"error": "No type-A robot found"},
            fallback_used=True,
        )

    # 按机器位置自然排序（先 row 后 x，而非字符串排序）
    from functools import partial
    sort_key = partial(_machine_sort_key, machines)
    machine_ids = sorted(machines.keys(), key=sort_key)

    # 收集各类型的操作
    disassemble_ops: list[str] = []
    inspect_ops: list[str] = []
    install_ops: list[str] = []

    for machine_id in machine_ids:
        d_op = f"{machine_id}_D"
        i_op = f"{machine_id}_I"
        r_op = f"{machine_id}_R"
        if d_op in operations:
            disassemble_ops.append(d_op)
        if i_op in operations:
            inspect_ops.append(i_op)
        if r_op in operations:
            install_ops.append(r_op)

    # A 机器人的操作: 全部 D + 全部 R（先D后R）
    a_operations = disassemble_ops + install_ops

    # B 机器人的操作: 全部 I
    b_operations = inspect_ops

    # 构建 RobotSchedule（时间在规划阶段确定，此处为-1占位）
    a_schedule = RobotSchedule(
        robot_id=robot_a,
        operations=[(op_id, -1, -1) for op_id in a_operations],
    )

    result = ScheduleResult(
        status=ResultStatus.FEASIBLE.value,
        objective={
            "robot_a_ops": len(a_operations),
            "robot_b_ops": len(b_operations) if robot_b else 0,
            "method": "manual_fallback",
        },
        fallback_used=True,
    )
    result.robot_schedules[robot_a] = a_schedule

    if robot_b:
        b_schedule = RobotSchedule(
            robot_id=robot_b,
            operations=[(op_id, -1, -1) for op_id in b_operations],
        )
        result.robot_schedules[robot_b] = b_schedule

    # 构建 assignments 列表
    for op_id in a_operations:
        result.assignments.append({
            "operation_id": op_id,
            "robot_id": robot_a,
            "machine_id": operations[op_id].machine_id,
            "operation_type": operations[op_id].operation_type.value,
        })
    for op_id in b_operations:
        result.assignments.append({
            "operation_id": op_id,
            "robot_id": robot_b,
            "machine_id": operations[op_id].machine_id,
            "operation_type": operations[op_id].operation_type.value,
        })

    return result


def manual_assign_multi_robot(
    machines: dict[str, Machine],
    operations: dict[str, Operation],
    robots: dict[str, RobotSpec],
) -> ScheduleResult:
    """多机器人场景手动分配（负载均衡分配）。

    场景2 (2A1B) 和场景3 (4A2B):
    - DISASSEMBLE 和 INSTALL 平均分配给所有 A 机器人
    - INSPECT 平均分配给所有 B 机器人

    Args:
        machines: 离心机字典
        operations: 操作字典
        robots: 机器人规格字典

    Returns:
        ScheduleResult 包含各机器人的操作序列
    """
    # 按类型分组机器人
    a_robots = [rid for rid, r in robots.items() if r.robot_type == RobotType.A]
    b_robots = [rid for rid, r in robots.items() if r.robot_type == RobotType.B]

    if not a_robots:
        return ScheduleResult(
            status=ResultStatus.INVALID_INPUT.value,
            objective={"error": "No type-A robots found"},
            fallback_used=True,
        )

    # 按机器位置自然排序（先 row 后 x，而非字符串排序）
    from functools import partial
    sort_key = partial(_machine_sort_key, machines)
    machine_ids = sorted(machines.keys(), key=sort_key)

    # 收集操作
    disassemble_ops: list[str] = []
    inspect_ops: list[str] = []
    install_ops: list[str] = []

    for machine_id in machine_ids:
        d_op = f"{machine_id}_D"
        i_op = f"{machine_id}_I"
        r_op = f"{machine_id}_R"
        if d_op in operations:
            disassemble_ops.append(d_op)
        if i_op in operations:
            inspect_ops.append(i_op)
        if r_op in operations:
            install_ops.append(r_op)

    result = ScheduleResult(
        status=ResultStatus.FEASIBLE.value,
        objective={"method": "manual_fallback"},
        fallback_used=True,
    )

    # 轮询分配给 A 机器人
    num_a = len(a_robots)
    for i, op_id in enumerate(disassemble_ops):
        rid = a_robots[i % num_a]
        if rid not in result.robot_schedules:
            result.robot_schedules[rid] = RobotSchedule(robot_id=rid)
        result.robot_schedules[rid].operations.append((op_id, -1, -1))
        result.assignments.append({
            "operation_id": op_id,
            "robot_id": rid,
            "machine_id": operations[op_id].machine_id,
            "operation_type": "DISASSEMBLE",
        })

    for i, op_id in enumerate(install_ops):
        rid = a_robots[i % num_a]
        if rid not in result.robot_schedules:
            result.robot_schedules[rid] = RobotSchedule(robot_id=rid)
        result.robot_schedules[rid].operations.append((op_id, -1, -1))
        result.assignments.append({
            "operation_id": op_id,
            "robot_id": rid,
            "machine_id": operations[op_id].machine_id,
            "operation_type": "INSTALL",
        })

    # 轮询分配给 B 机器人
    if b_robots:
        num_b = len(b_robots)
        for i, op_id in enumerate(inspect_ops):
            rid = b_robots[i % num_b]
            if rid not in result.robot_schedules:
                result.robot_schedules[rid] = RobotSchedule(robot_id=rid)
            result.robot_schedules[rid].operations.append((op_id, -1, -1))
            result.assignments.append({
                "operation_id": op_id,
                "robot_id": rid,
                "machine_id": operations[op_id].machine_id,
                "operation_type": "INSPECT",
            })

    # 统计
    a_loads = {
        rid: len(sched.operations)
        for rid, sched in result.robot_schedules.items()
        if rid in a_robots
    }
    b_loads = {
        rid: len(sched.operations)
        for rid, sched in result.robot_schedules.items()
        if rid in b_robots
    }
    result.objective["a_loads"] = a_loads
    result.objective["b_loads"] = b_loads

    return result


def manual_assign(
    problem: SchedulingProblem,
) -> ScheduleResult:
    """根据机器人和操作数自动选择分配策略。

    Args:
        problem: 调度问题（包含机器、操作、机器人列表）

    Returns:
        ScheduleResult
    """
    a_count = sum(
        1 for r in problem.robots.values() if r.robot_type == RobotType.A
    )
    b_count = sum(
        1 for r in problem.robots.values() if r.robot_type == RobotType.B
    )

    if a_count == 1 and b_count <= 1:
        return manual_assign_scenario_1(
            problem.machines, problem.operations, problem.robots
        )
    else:
        return manual_assign_multi_robot(
            problem.machines, problem.operations, problem.robots
        )
