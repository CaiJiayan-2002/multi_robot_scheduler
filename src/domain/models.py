"""领域数据模型 v4.0"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

from .enums import RobotType, MachineState, OperationType, Action


@dataclass(frozen=True)
class Cell:
    """栅格坐标 (x=col, y=row)，原点左上角。(1-indexed)"""
    x: int
    y: int

    def __add__(self, other: Cell) -> Cell:
        return Cell(self.x + other.x, self.y + other.y)


@dataclass(frozen=True)
class Footprint:
    """机器人本体 2x4 的偏移列表"""
    offsets: tuple[Cell, ...]

    @staticmethod
    def default_2x4() -> Footprint:
        """默认 A/B 机器人 footprint：宽 2，高 4"""
        offsets = tuple(
            Cell(dx, dy)
            for dy in range(4)
            for dx in range(2)
        )
        return Footprint(offsets)

    @staticmethod
    def work_zone() -> tuple[Cell, Cell]:
        """工作区：本体最上方两个格"""
        return (Cell(0, 0), Cell(1, 0))

    def cells_at(self, anchor: Cell) -> frozenset[Cell]:
        """给定锚点，返回所有占用格"""
        return frozenset(anchor + off for off in self.offsets)


@dataclass(frozen=True)
class Operation:
    """单个作业操作"""
    operation_id: str
    machine_id: str
    operation_type: OperationType
    eligible_robot_type: RobotType
    duration: int
    service_anchor: Cell
    predecessor_id: str | None = None


@dataclass
class Machine:
    """单台离心机（2格横向）"""
    machine_id: str
    cells: tuple[Cell, Cell]
    row: int
    state: MachineState = MachineState.PENDING_DISASSEMBLY
    locked_by: str | None = None


@dataclass
class RobotSpec:
    """机器人规格"""
    robot_id: str
    robot_type: RobotType
    start_anchor: Cell
    footprint: Footprint = field(default_factory=Footprint.default_2x4)


@dataclass(frozen=True)
class TimedPose:
    """带时间的机器人姿态"""
    t: int
    x: int
    y: int
    action: str
    operation_id: str | None = None


@dataclass
class RobotSchedule:
    """求解器输出的机器人操作序列"""
    robot_id: str
    operations: list[tuple[str, int, int]] = field(default_factory=list)
    # (operation_id, planned_start, planned_end)


@dataclass
class ScheduleResult:
    """求解器输出"""
    status: str
    objective: dict = field(default_factory=dict)
    solve_time_seconds: float = 0.0
    best_objective_bound: float = 0.0
    assignments: list[dict] = field(default_factory=list)
    robot_schedules: dict[str, RobotSchedule] = field(default_factory=dict)
    fallback_used: bool = False


@dataclass
class SchedulingProblem:
    """求解器输入"""
    machines: dict[str, Machine]
    operations: dict[str, Operation]
    robots: dict[str, RobotSpec]
    travel_times: dict = field(default_factory=dict)
