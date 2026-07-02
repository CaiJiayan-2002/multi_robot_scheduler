"""离散事件仿真引擎 v4.0

整合状态机 + 预约表 + 时空A* + 求解结果，驱动多机器人协同调度仿真。

每个时间步:
1. 处理外部事件（延迟/障碍）
2. 为空闲机器人规划下一任务路径
3. 校验每台机器人下一动作
4. 同步执行所有动作（先提交预约，再统一执行）
5. 更新位置
6. 更新作业进度
7. 作业结束时更新机器状态
8. 检查碰撞
9. 记录事件日志
"""

from __future__ import annotations

from dataclasses import dataclass, field

from ..domain.enums import (
    Action, MachineState, OperationType, RobotStatus, RobotType, ResultStatus,
)
from ..domain.models import (
    Cell, Footprint, Machine, Operation, RobotSpec, ScheduleResult, TimedPose,
)
from ..domain.validation import FootprintValidator
from ..map.fixed_map import FixedMap
from ..map.pose_graph import PoseGraph
from ..map.service_poses import ServicePoseCalculator
from ..planning.reservation_table import ReservationTable
from ..planning.space_time_astar import SpaceTimeAStar
from ..planning.static_astar import StaticAStar
from ..solver.fallback import manual_assign
from .state_machine import MachineStateMachine

import numpy as np


# ======================================================================
# 机器人运行时状态
# ======================================================================

@dataclass
class RobotRuntime:
    """机器人在仿真中的运行时状态。

    Attributes:
        spec: 机器人静态规格
        status: 当前状态
        current_anchor: 当前锚点位置
        current_path: 当前规划的路径
        path_index: 路径中的当前位置索引
        current_op_id: 当前执行的操作ID
        work_remaining: 剩余工作时间（0=不在工作）
        assigned_ops: 待执行的操作ID列表
        completed_ops: 已完成的操作ID列表
        finished: 是否已完成所有分配
    """
    spec: RobotSpec
    status: RobotStatus = RobotStatus.IDLE
    current_anchor: Cell | None = None
    current_path: list[TimedPose] | None = None
    path_index: int = 0
    current_op_id: str | None = None
    work_remaining: int = 0
    assigned_ops: list[str] = field(default_factory=list)
    completed_ops: list[str] = field(default_factory=list)
    finished: bool = False


# ======================================================================
# 仿真引擎
# ======================================================================

class SimulationEngine:
    """离散时间仿真器。

    驱动多机器人系统的分步执行：
    - 读取任务分配结果
    - 规划路径并预约时空资源
    - 逐步推进仿真

    Attributes:
        current_time: 当前仿真时间
        terrain: 地形矩阵
        machines: 离心机字典
        operations: 操作字典
        robots: 机器人运行时状态字典
        state_machine: 机器状态机
        reservation_table: 时空预约表
        pose_graph: 姿态图
        static_astar: 静态 A* 规划器
        space_time_astar: 时空 A* 规划器
        footprint: 机器人 footprint
        service_anchors: {machine_id: service_anchor}
        event_log: 事件日志列表
    """

    def __init__(
        self,
        scenario_config: dict | None = None,
    ) -> None:
        """初始化仿真引擎。

        Args:
            scenario_config: 场景配置字典（机器人起始位置等）
        """
        self.scenario_config = scenario_config or {}
        self.current_time: int = 0

        # 地图数据（由 setup() 填充）
        self.terrain: np.ndarray | None = None
        self.machines: dict[str, Machine] = {}
        self.operations: dict[str, Operation] = {}
        self.robot_specs: dict[str, RobotSpec] = {}

        # 运行时状态
        self.robots: dict[str, RobotRuntime] = {}
        self.state_machine: MachineStateMachine | None = None
        self.footprint: Footprint = Footprint.default_2x4()

        # 规划组件
        self.reservation_table: ReservationTable = ReservationTable()
        self.pose_graph: PoseGraph | None = None
        self.static_astar: StaticAStar | None = None
        self.space_time_astar: SpaceTimeAStar | None = None
        self.service_anchors: dict[str, Cell] = {}

        # 事件日志
        self.event_log: list[dict] = []

        # 仿真状态
        self._running: bool = False
        self._max_steps: int = 20000

    # ==================================================================
    # 初始化
    # ==================================================================

    def setup(
        self,
        terrain: np.ndarray,
        machines: dict[str, Machine],
        operations: dict[str, Operation],
        robots: dict[str, RobotSpec],
        schedule: ScheduleResult,
    ) -> None:
        """用求解结果初始化仿真。

        构建姿态图、规划器、服务锚点映射，
        并为每个机器人创建运行时状态。

        Args:
            terrain: 2D numpy 地形矩阵
            machines: 离心机字典
            operations: 操作字典
            robots: 机器人规格字典
            schedule: 任务分配结果
        """
        self.terrain = terrain
        self.machines = machines
        self.operations = operations
        self.robot_specs = robots
        self.current_time = 0
        self.event_log.clear()

        # 构建姿态图
        self.pose_graph = PoseGraph(terrain, self.footprint)
        self.pose_graph.build()

        # 构建规划器
        self.static_astar = StaticAStar(self.pose_graph)
        self.space_time_astar = SpaceTimeAStar(
            self.pose_graph, self.reservation_table, self.footprint
        )

        # 计算服务锚点
        self.service_anchors = ServicePoseCalculator.compute_all_service_anchors(
            machines, self.footprint
        )

        # 初始化状态机（深拷贝机器状态，避免污染原始数据）
        machine_copies = {
            mid: Machine(
                machine_id=m.machine_id,
                cells=m.cells,
                row=m.row,
                state=m.state,
                locked_by=None,
            )
            for mid, m in machines.items()
        }
        self.state_machine = MachineStateMachine(machine_copies)

        # 创建机器人运行时
        self.robots.clear()
        for rid, rspec in robots.items():
            runtime = RobotRuntime(
                spec=rspec,
                status=RobotStatus.IDLE,
                current_anchor=rspec.start_anchor,
            )

            # 从 schedule 中获取分配的操作
            if rid in schedule.robot_schedules:
                rs = schedule.robot_schedules[rid]
                runtime.assigned_ops = [op_id for op_id, _, _ in rs.operations]

            self.robots[rid] = runtime

            # 预约起始位置
            start_cells = self.footprint.cells_at(rspec.start_anchor)
            self.reservation_table.reserve_pose(0, start_cells, rid)

        self._running = True
        self._log_event("setup", "Simulation initialized")

    def setup_scenario_1(self) -> None:
        """快速搭建场景1（1A1B）的完整仿真环境。

        使用 FixedMap 生成地图，manual_assign 生成任务分配。
        """
        # 生成地图
        fixed_map = FixedMap()
        terrain, machines, operations = fixed_map.build()

        # 场景1机器人
        robots = {
            "A_1": RobotSpec(
                robot_id="A_1",
                robot_type=RobotType.A,
                start_anchor=Cell(1, 24),  # 主干道左侧
            ),
            "B_1": RobotSpec(
                robot_id="B_1",
                robot_type=RobotType.B,
                start_anchor=Cell(24, 24),  # 主干道右侧
            ),
        }

        # 生成任务分配
        schedule = manual_assign(
            self._make_problem(machines, operations, robots)
        )

        self.setup(terrain, machines, operations, robots, schedule)

    # ==================================================================
    # 仿真主循环
    # ==================================================================

    def step(self) -> dict:
        """推进一个时间步，返回当前状态摘要。

        执行顺序:
        1. 为 IDLE 机器人规划下一任务
        2. 校验并提交所有机器人的下一步动作到预约表
        3. 同步执行所有动作
        4. 更新位置和工作进度
        5. 更新机器状态
        6. 时间推进

        Returns:
            当前状态摘要字典
        """
        if not self._running:
            return {"time": self.current_time, "status": "stopped", "event": None}

        self._log_event("step_start", f"Step {self.current_time} start")

        # 1. 为 IDLE 机器人规划下一任务路径
        self._plan_idle_robots()

        # 2. 收集所有机器人下一步动作
        next_actions = self._collect_next_actions()

        # 3. 提交动作到预约表并执行
        self._execute_actions(next_actions)

        # 4. 更新工作进度
        self._update_work_progress()

        # 5. 推进时间
        self.current_time += 1

        # 6. 检查终止条件（机器全完成 + 机器人全归位）
        if self.state_machine.all_completed():
            all_home = all(r.finished for r in self.robots.values())
            if all_home:
                self._running = False
                self._log_event("simulation_end",
                    "All centrifuges completed and all robots returned to start")

        if self.current_time >= self._max_steps:
            self._running = False
            self._log_event("simulation_end", f"Reached max steps {self._max_steps}")

        return self.get_state()

    def run(self, max_steps: int = 5000) -> list[dict]:
        """运行仿真直到终止条件（所有机器状态=10）。

        Args:
            max_steps: 最大步数限制

        Returns:
            事件日志列表
        """
        self._max_steps = max_steps
        self._running = True
        step_count = 0

        while self._running and step_count < max_steps:
            self.step()
            step_count += 1

        return self.event_log

    # ==================================================================
    # 内部方法：规划
    # ==================================================================

    def _plan_idle_robots(self) -> None:
        """为所有 IDLE 状态的机器人规划下一个任务的路径。"""
        for rid, robot in self.robots.items():
            if robot.status != RobotStatus.IDLE or robot.finished:
                continue

            if not robot.assigned_ops:
                # *** 所有任务完成：规划返回起点 ***
                self._plan_return_to_start(rid, robot)
                continue

            # 获取下一个操作
            next_op_id = robot.assigned_ops[0]
            operation = self.operations.get(next_op_id)
            if operation is None:
                self._log_event("error", f"{rid}: operation {next_op_id} not found")
                robot.assigned_ops.pop(0)
                continue

            # 检查机器是否准备好
            can_start, reason = self.state_machine.can_start_operation(
                operation.machine_id, operation.operation_type, rid
            )
            if not can_start:
                # 机器还没准备好，跳过此操作，尝试下一个
                self._log_event("wait_precedence",
                    f"{rid}: operation {next_op_id} cannot start: {reason}")
                # 如果机器人有多个操作，尝试跳过这个
                # 前序关系未满足时，标记为等待
                robot.status = RobotStatus.WAITING_PRECEDENCE
                continue

            # 尝试锁定机器
            if not self.state_machine.lock_machine(operation.machine_id, rid):
                self._log_event("wait_conflict",
                    f"{rid}: cannot lock machine {operation.machine_id}")
                robot.status = RobotStatus.WAITING_CONFLICT
                continue

            # 获取目标服务锚点
            goal_anchor = self.service_anchors.get(operation.machine_id)
            if goal_anchor is None:
                self._log_event("error",
                    f"machine {operation.machine_id} has no service anchor")
                self.state_machine.unlock_machine(operation.machine_id)
                continue

            # 规划路径（含服务时间）
            path = self.space_time_astar.plan_with_service(
                start_anchor=robot.current_anchor,
                goal_anchor=goal_anchor,
                start_time=self.current_time,
                service_duration=operation.duration,
                machine_id=operation.machine_id,
                operation_id=next_op_id,
                max_time=self._max_steps,
                robot_id=rid,
            )

            if path is None:
                self._log_event("planning_failed",
                    f"{rid}: cannot plan path to {operation.machine_id}")
                self.state_machine.unlock_machine(operation.machine_id)
                robot.status = RobotStatus.WAITING_CONFLICT
                continue

            # 预约路径
            if not self.space_time_astar.reserve_path(
                path, rid, operation.machine_id
            ):
                self._log_event("reservation_failed",
                    f"{rid}: path reservation failed")
                self.state_machine.unlock_machine(operation.machine_id)
                robot.status = RobotStatus.WAITING_CONFLICT
                continue

            # 设置运行时状态
            robot.current_path = path
            robot.path_index = 0
            robot.current_op_id = next_op_id
            robot.status = RobotStatus.MOVING
            robot.assigned_ops.pop(0)

            self._log_event("planned",
                f"{rid}: planned path to {operation.machine_id} "
                f"({operation.operation_type.value}), "
                f"arrival_time={path[-1].t}, path_len={len(path)}")

    # ==================================================================
    # 内部方法：返回起点
    # ==================================================================

    def _plan_return_to_start(self, rid: str, robot: RobotRuntime) -> None:
        """所有任务完成，规划返回起点路径。"""
        start_anchor = robot.spec.start_anchor
        cur = robot.current_anchor

        if cur is None or cur == start_anchor:
            robot.finished = True
            robot.status = RobotStatus.FINISHED
            self._log_event("robot_finished",
                f"{rid}: all done, already at start")
            return

        path = self.space_time_astar.plan(
            start_anchor=cur,
            goal_anchor=start_anchor,
            start_time=self.current_time,
            max_time=self._max_steps,
            robot_id=rid,
        )

        if path is None:
            robot.finished = True
            robot.status = RobotStatus.FINISHED
            self._log_event("robot_finished",
                f"{rid}: cannot return to start, finishing at ({cur.x},{cur.y})")
            return

        if not self.space_time_astar.reserve_path(path, rid, None):
            robot.finished = True
            robot.status = RobotStatus.FINISHED
            self._log_event("robot_finished",
                f"{rid}: cannot reserve return path")
            return

        robot.current_path = path
        robot.path_index = 0
        robot.current_op_id = "__RETURN__"
        robot.status = RobotStatus.MOVING
        self._log_event("return_to_start",
            f"{rid}: returning to start ({start_anchor.x},{start_anchor.y}), "
            f"path={len(path)} steps, arrival_t={path[-1].t}")

    # ==================================================================
    # 内部方法：动作收集与执行
    # ==================================================================

    def _collect_next_actions(self) -> dict[str, dict]:
        """收集所有机器人下一步的动作信息。

        Returns:
            {robot_id: {pose, action, is_working, ...}} 字典
        """
        actions: dict[str, dict] = {}

        for rid, robot in self.robots.items():
            if robot.status in (RobotStatus.IDLE, RobotStatus.FINISHED):
                continue

            if robot.status == RobotStatus.WAITING_PRECEDENCE:
                # 重新检查前置条件
                if robot.assigned_ops:
                    next_op_id = robot.assigned_ops[0]
                    op = self.operations.get(next_op_id)
                    if op:
                        can_start, _ = self.state_machine.can_start_operation(
                            op.machine_id, op.operation_type, rid
                        )
                        if can_start:
                            robot.status = RobotStatus.IDLE
                            self._log_event("precedence_cleared",
                                f"{rid}: precedence cleared, re-entering IDLE")
                if robot.status == RobotStatus.WAITING_PRECEDENCE:
                    actions[rid] = {"action_type": "wait", "reason": "precedence"}
                    continue

            if robot.status == RobotStatus.WAITING_CONFLICT:
                # 清除预约，重新尝试
                self.reservation_table.release_future(rid, self.current_time)
                robot.status = RobotStatus.IDLE
                self._log_event("retry", f"{rid}: clearing reservations, retrying")
                actions[rid] = {"action_type": "wait", "reason": "conflict"}
                continue

            if robot.current_path is None or robot.path_index >= len(robot.current_path):
                # 路径结束
                if robot.work_remaining > 0:
                    robot.status = RobotStatus.WORKING
                    actions[rid] = {"action_type": "working", "remaining": robot.work_remaining}
                else:
                    robot.status = RobotStatus.IDLE
                    actions[rid] = {"action_type": "idle"}
                continue

            # 获取当前路径点
            current_pose = robot.current_path[robot.path_index]
            next_t = current_pose.t

            if next_t > self.current_time:
                # 未来时间点，不移动（已经预约但时间未到）
                actions[rid] = {"action_type": "wait", "reason": "scheduled"}
                continue

            actions[rid] = {
                "action_type": "move",
                "pose": current_pose,
                "path_index": robot.path_index,
            }

        return actions

    def _execute_actions(self, actions: dict[str, dict]) -> None:
        """执行所有机器人的下一步动作。"""
        for rid, action in actions.items():
            robot = self.robots.get(rid)
            if robot is None:
                continue

            atype = action.get("action_type", "unknown")

            if atype == "move":
                pose = action["pose"]
                robot.current_anchor = Cell(pose.x, pose.y)
                robot.path_index = action["path_index"] + 1

                # 检查是否到达路径末尾
                if robot.path_index >= len(robot.current_path):
                    # 返回起点完成
                    if robot.current_op_id == "__RETURN__":
                        robot.finished = True
                        robot.status = RobotStatus.FINISHED
                        robot.current_op_id = None
                        robot.current_path = None
                        self._log_event("robot_finished",
                            f"{rid}: arrived at start, all tasks done")
                    # 作业从下一个时间步开始
                    elif robot.current_op_id:
                        op = self.operations.get(robot.current_op_id)
                        if op:
                            robot.work_remaining = op.duration
                            robot.status = RobotStatus.WORKING
                            self._log_event("work_start",
                                f"{rid}: start {op.operation_type.value} "
                                f"on {op.machine_id}, duration={op.duration}")
                    else:
                        robot.status = RobotStatus.IDLE

                self._log_event("move",
                    f"{rid}: -> ({pose.x},{pose.y}) t={pose.t} {pose.action}")

            elif atype == "working":
                robot.status = RobotStatus.WORKING

            elif atype == "idle":
                robot.status = RobotStatus.IDLE

            elif atype == "wait":
                pass  # 保持当前状态

    # ==================================================================
    # 内部方法：工作进度更新
    # ==================================================================

    def _update_work_progress(self) -> None:
        """更新所有 WORKING 机器人的工作进度，完成时推进机器状态。"""
        for rid, robot in self.robots.items():
            if robot.status != RobotStatus.WORKING:
                continue

            if robot.work_remaining > 0:
                robot.work_remaining -= 1
                self._log_event("work_tick",
                    f"{rid}: work remaining={robot.work_remaining}")

            if robot.work_remaining <= 0:
                # 工作完成
                op_id = robot.current_op_id
                if op_id and op_id in self.operations:
                    op = self.operations[op_id]
                    # 推进机器状态
                    self.state_machine.complete_operation(
                        op.machine_id, op.operation_type
                    )
                    # 解锁机器
                    self.state_machine.unlock_machine(op.machine_id)

                    self._log_event("work_complete",
                        f"{rid}: completed {op.operation_type.value} "
                        f"on {op.machine_id}, "
                        f"machine state={self.state_machine.machines[op.machine_id].state.name}")

                robot.completed_ops.append(op_id if op_id else "")
                robot.current_op_id = None
                robot.current_path = None
                robot.status = RobotStatus.IDLE

    # ==================================================================
    # 状态查询
    # ==================================================================

    def get_state(self) -> dict:
        """返回当前仿真状态摘要。

        Returns:
            包含时间、机器人状态、离心机状态统计的字典
        """
        robot_states = {}
        for rid, robot in self.robots.items():
            robot_states[rid] = {
                "status": robot.status.name,
                "anchor": (
                    (robot.current_anchor.x, robot.current_anchor.y)
                    if robot.current_anchor else None
                ),
                "current_op": robot.current_op_id,
                "work_remaining": robot.work_remaining,
                "completed": len(robot.completed_ops),
                "assigned": len(robot.assigned_ops),
            }

        machine_summary = self.state_machine.summary() if self.state_machine else {}

        return {
            "time": self.current_time,
            "running": self._running,
            "robots": robot_states,
            "machines": machine_summary,
            "events_this_step": len(
                [e for e in self.event_log if e.get("t") == self.current_time]
            ),
        }

    def is_finished(self) -> bool:
        """检查仿真是否已结束。"""
        if self.state_machine is None:
            return False
        return self.state_machine.all_completed() or not self._running

    # ==================================================================
    # 事件日志
    # ==================================================================

    def _log_event(self, event_type: str, message: str) -> None:
        """记录事件到日志。

        Args:
            event_type: 事件类型
            message: 事件描述
        """
        self.event_log.append({
            "t": self.current_time,
            "type": event_type,
            "message": message,
        })

    def get_events_by_type(self, event_type: str) -> list[dict]:
        """按类型过滤事件日志。

        Args:
            event_type: 事件类型

        Returns:
            过滤后的事件列表
        """
        return [e for e in self.event_log if e.get("type") == event_type]

    # ==================================================================
    # 辅助方法
    # ==================================================================

    @staticmethod
    def _make_problem(
        machines: dict[str, Machine],
        operations: dict[str, Operation],
        robots: dict[str, RobotSpec],
    ) -> object:
        """创建 SchedulingProblem 辅助函数。

        Args:
            machines: 离心机字典
            operations: 操作字典
            robots: 机器人规格字典

        Returns:
            SchedulingProblem 实例
        """
        from ..domain.models import SchedulingProblem
        return SchedulingProblem(
            machines=machines,
            operations=operations,
            robots=robots,
        )
