"""离心机状态机 v4.0

管理48台离心机的状态转换: 7(PENDING_DISASSEMBLY) -> 8(PENDING_INSPECTION) ->
9(PENDING_INSTALLATION) -> 10(COMPLETED)

关键规则:
- 状态只能在作业结束时改变
- 作业期间机器必须加锁 (locked_by = robot_id)
- 不能跳过状态
- 不能同时被多个机器人操作(同一时间最多一个机器人锁定)
- D 由 A 机器人执行，I 由 B 机器人执行，R 由 A 机器人执行
"""

from __future__ import annotations

from ..domain.enums import MachineState, OperationType, RobotType
from ..domain.models import Machine


# 操作类型 -> 前置机器状态
_OP_PREREQUISITE: dict[OperationType, MachineState] = {
    OperationType.DISASSEMBLE: MachineState.PENDING_DISASSEMBLY,
    OperationType.INSPECT: MachineState.PENDING_INSPECTION,
    OperationType.INSTALL: MachineState.PENDING_INSTALLATION,
}

# 操作类型 -> 完成后机器应进入的状态
_OP_NEXT_STATE: dict[OperationType, MachineState] = {
    OperationType.DISASSEMBLE: MachineState.PENDING_INSPECTION,
    OperationType.INSPECT: MachineState.PENDING_INSTALLATION,
    OperationType.INSTALL: MachineState.COMPLETED,
}

# 操作类型 -> 可执行的机器人类型
_OP_ROBOT_TYPE: dict[OperationType, RobotType] = {
    OperationType.DISASSEMBLE: RobotType.A,
    OperationType.INSPECT: RobotType.B,
    OperationType.INSTALL: RobotType.A,
}


class MachineStateMachine:
    """管理48台离心机的状态转换。

    每台离心机经历 DISASSEMBLE -> INSPECT -> INSTALL 三个操作链，
    状态从 7 逐步推进到 10。

    Attributes:
        machines: 离心机字典 {machine_id: Machine}
    """

    def __init__(self, machines: dict[str, Machine]) -> None:
        """初始化状态机。

        Args:
            machines: 离心机字典，key=machine_id
        """
        self.machines = machines

    # ------------------------------------------------------------------
    def can_start_operation(
        self,
        machine_id: str,
        operation_type: OperationType,
        robot_id: str | None = None,
    ) -> tuple[bool, str]:
        """检查某操作现在是否可以开始。

        检查项:
        1. 机器状态是否为该操作的前置状态
        2. 机器是否已被锁定
        3. (可选) 锁定者是否为当前机器人

        Args:
            machine_id: 离心机ID
            operation_type: 操作类型
            robot_id: 请求的机器人ID（可选，用于验证锁持有者）

        Returns:
            (can_start, reason) 元组；can_start=True 表示可以开始
        """
        machine = self.machines.get(machine_id)
        if machine is None:
            return False, f"机器 {machine_id} 不存在"

        # 检查机器状态
        required_state = _OP_PREREQUISITE.get(operation_type)
        if required_state is None:
            return False, f"未知操作类型: {operation_type}"

        if machine.state != required_state:
            return (
                False,
                f"机器 {machine_id} 当前状态为 {machine.state.name}，"
                f"需要 {required_state.name} 才能执行 {operation_type.name}",
            )

        # 检查是否被锁定
        if machine.locked_by is not None:
            if robot_id and machine.locked_by == robot_id:
                return True, "OK（当前机器人持有锁）"
            return (
                False,
                f"机器 {machine_id} 已被 {machine.locked_by} 锁定",
            )

        return True, "OK"

    # ------------------------------------------------------------------
    def lock_machine(self, machine_id: str, robot_id: str) -> bool:
        """尝试锁定机器。只有未被锁定的机器才能被锁定。

        Args:
            machine_id: 离心机ID
            robot_id: 请求锁定的机器人ID

        Returns:
            True 如果锁定成功，False 如果已被其他机器人锁定
        """
        machine = self.machines.get(machine_id)
        if machine is None:
            return False

        if machine.locked_by is None:
            machine.locked_by = robot_id
            return True

        # 已被同一机器人锁定 -> 允许（幂等）
        if machine.locked_by == robot_id:
            return True

        return False

    # ------------------------------------------------------------------
    def unlock_machine(self, machine_id: str) -> None:
        """释放对机器的锁定。

        Args:
            machine_id: 离心机ID
        """
        machine = self.machines.get(machine_id)
        if machine is not None:
            machine.locked_by = None

    # ------------------------------------------------------------------
    def complete_operation(self, machine_id: str, operation_type: OperationType) -> None:
        """操作完成，将机器状态推进到下一状态。

        验证完成后调用此方法，状态从 7->8->9->10 逐步推进。

        Args:
            machine_id: 离心机ID
            operation_type: 完成的操作类型

        Raises:
            ValueError: 如果状态转换无效
        """
        machine = self.machines.get(machine_id)
        if machine is None:
            raise ValueError(f"机器 {machine_id} 不存在")

        expected_next = _OP_NEXT_STATE.get(operation_type)
        if expected_next is None:
            raise ValueError(f"未知操作类型: {operation_type}")

        machine.state = expected_next

    # ------------------------------------------------------------------
    def get_machines_ready_for(
        self, operation_type: OperationType
    ) -> list[str]:
        """获取所有可以立即执行某操作的机器ID。

        条件: 机器状态匹配操作前置状态 且 未被锁定。

        Args:
            operation_type: 操作类型

        Returns:
            机器ID列表
        """
        required_state = _OP_PREREQUISITE.get(operation_type)
        if required_state is None:
            return []

        ready = []
        for machine_id, machine in self.machines.items():
            if machine.state == required_state and machine.locked_by is None:
                ready.append(machine_id)
        return ready

    # ------------------------------------------------------------------
    def get_machines_in_state(self, state: MachineState) -> list[str]:
        """获取所有处于指定状态的机器ID。

        Args:
            state: 目标状态

        Returns:
            机器ID列表
        """
        return [
            mid for mid, m in self.machines.items() if m.state == state
        ]

    # ------------------------------------------------------------------
    def all_completed(self) -> bool:
        """检查是否所有48台离心机都达到状态10（COMPLETED）。

        Returns:
            True 如果全部完成
        """
        return all(
            m.state == MachineState.COMPLETED
            for m in self.machines.values()
        )

    # ------------------------------------------------------------------
    def completed_count(self) -> int:
        """返回已完成（状态=10）的机器数量。

        Returns:
            已完成的机器数
        """
        return sum(
            1 for m in self.machines.values()
            if m.state == MachineState.COMPLETED
        )

    # ------------------------------------------------------------------
    def summary(self) -> dict:
        """返回状态分布的摘要统计。

        Returns:
            {状态名: 机器数量} 字典
        """
        counts: dict[str, int] = {}
        for state in MachineState:
            count = sum(
                1 for m in self.machines.values() if m.state == state
            )
            counts[state.name] = count
        counts["total"] = len(self.machines)
        return counts
