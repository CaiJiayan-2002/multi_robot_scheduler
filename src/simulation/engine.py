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
from ..planning.conflicts import PlanningConflict
from ..solver.scheduler import solve_assignment_schedule
from ..solver.config import SolverConfig
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
    retry_after: int = 0
    yield_requested: bool = False
    planned_start_by_op: dict[str, int] = field(default_factory=dict)


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
        self.planning_conflicts: list[PlanningConflict] = []

        # 仿真状态
        self._running: bool = False
        self._max_steps: int = 20000
        # 单次局部规划只看有限未来；不可达时快速重试，避免在时间维上
        # 一直展开到整场仿真的结束时刻。
        self._planning_window: int = 64

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
        self.planning_conflicts.clear()
        self.reservation_table = ReservationTable()

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
                runtime.planned_start_by_op = {
                    op_id: planned_start
                    for op_id, planned_start, _ in rs.operations
                }

            self.robots[rid] = runtime

            # 预约起始位置
            start_cells = self.footprint.cells_at(rspec.start_anchor)
            self.reservation_table.reserve_pose(0, start_cells, rid)

        self._running = True
        self._log_event("setup", "Simulation initialized")

    def setup_scenario_1(self) -> None:
        """快速搭建场景1（1A1B）的完整仿真环境。

        使用 FixedMap 与完整 CP-SAT 生成任务分配、顺序和计划时间。
        """
        # 生成地图
        fixed_map = FixedMap()
        terrain, machines, operations = fixed_map.build()

        # 场景1机器人
        robots = {
            "A_1": RobotSpec(
                robot_id="A_1",
                robot_type=RobotType.A,
                start_anchor=Cell(1, 28),  # 第28行左侧停车位
            ),
            "B_1": RobotSpec(
                robot_id="B_1",
                robot_type=RobotType.B,
                start_anchor=Cell(24, 28),  # 第28行右侧停车位
            ),
        }

        # 生成任务分配
        schedule = solve_assignment_schedule(
            terrain, machines, operations, robots,
            SolverConfig(allow_fallback=False),
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
        self.reservation_table.prune_before(self.current_time)

        # 1. 为 IDLE 机器人规划下一任务路径
        self._plan_idle_robots()

        # 2. 收集所有机器人下一步动作
        next_actions = self._collect_next_actions()
        next_actions = self._filter_unsafe_actions(next_actions)

        # 3. 提交动作到预约表并执行
        self._execute_actions(next_actions)

        # *** 碰撞检测 ***
        collisions = self._check_collisions()
        if collisions:
            for c in collisions:
                self._log_event("collision", c)

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
    # 碰撞检测
    # ==================================================================

    def _check_collisions(self) -> list[str]:
        """检查所有机器人对之间的 footprint 重叠。

        Returns:
            碰撞描述列表（空列表 = 无碰撞）
        """
        collisions: list[str] = []
        robot_list = [(rid, r) for rid, r in self.robots.items()
                      if r.current_anchor is not None and not r.finished]

        for i in range(len(robot_list)):
            rid_a, robot_a = robot_list[i]
            cells_a = self.footprint.cells_at(robot_a.current_anchor)
            for j in range(i + 1, len(robot_list)):
                rid_b, robot_b = robot_list[j]
                cells_b = self.footprint.cells_at(robot_b.current_anchor)
                overlap = cells_a & cells_b
                if overlap:
                    overlap_list = [(c.x, c.y) for c in overlap]
                    collisions.append(
                        f"COLLISION t={self.current_time}: {rid_a}@{robot_a.current_anchor} "
                        f"vs {rid_b}@{robot_b.current_anchor}, "
                        f"cells={overlap_list}"
                    )
        return collisions

    # ==================================================================
    # 内部方法：规划
    # ==================================================================

    def _plan_idle_robots(self) -> None:
        """为所有 IDLE 状态的机器人规划下一个任务的路径。"""
        # *** 注入静止机器人位置到预约表 ***
        self._inject_static_robots()

        # 正常情况下允许多机器人同时规划/移动。时空预约表和执行前
        # safety guard 负责避免本体碰撞；如果仍保留“单移动令牌”，
        # 2A1B 会被间接串行化，难以形成 A1/A2/B1 同时作业的流水线。
        multi_robot_motion_busy = False
        pending_yield = any(
            runtime.yield_requested and not runtime.finished
            for runtime in self.robots.values()
        )

        for rid, robot in self.robots.items():
            if robot.status != RobotStatus.IDLE or robot.finished:
                continue
            if len(self.robots) > 2 and pending_yield and not robot.yield_requested:
                continue
            if multi_robot_motion_busy:
                continue

            # B 在与执行任务的 A 发生路径冲突后，先驶回右侧停车位，
            # 到位后再恢复原任务。A 的任务因此获得明确通行优先级。
            if robot.yield_requested:
                if self._plan_yield_to_home(rid, robot):
                    continue

            if not robot.assigned_ops:
                # *** 所有任务完成：规划返回起点 ***
                self._plan_return_to_start(rid, robot)
                if (
                    len(self.robots) > 2
                    and robot.status == RobotStatus.MOVING
                ):
                    pass
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

            # 规划路径
            # 作业约束：换列必须经过主干道
            same_column = (robot.current_anchor.x == goal_anchor.x)
            if same_column:
                path = self.space_time_astar.plan(
                    start_anchor=robot.current_anchor,
                    goal_anchor=goal_anchor,
                    start_time=self.current_time,
                    max_time=min(self._max_steps, self.current_time + self._planning_window),
                    robot_id=rid,
                )
            else:
                trunk_y = self.pose_graph.trunk_y_threshold
                trunk_wp = Cell(goal_anchor.x, trunk_y)
                path = self._plan_two_legs(robot.current_anchor, trunk_wp, goal_anchor, self.current_time, rid)

            if path is None:
                blockers = [
                    other.current_op_id
                    for other_id, other in self.robots.items()
                    if other_id != rid and other.current_op_id
                ]
                conflict_ops = tuple([next_op_id] + blockers)
                suggested = (
                    (blockers[0], next_op_id, 1) if blockers else None
                )
                self.planning_conflicts.append(PlanningConflict(
                    robot_id=rid,
                    conflicting_operation_ids=conflict_ops,
                    conflicting_time_interval=(
                        self.current_time,
                        self.current_time + self._planning_window,
                    ),
                    minimum_required_delay=1,
                    suggested_precedence_constraint=suggested,
                ))
                self._log_event("planning_failed",
                    f"{rid}: cannot plan path to {operation.machine_id}")
                self.state_machine.unlock_machine(operation.machine_id)
                robot.status = RobotStatus.WAITING_CONFLICT
                continue

            # CP-SAT 的 start 是服务开始下界。若静态计划留有时间余量，
            # 在目标位等待，路径规划器不得提前改变任务顺序或开工作业。
            planned_start = robot.planned_start_by_op.get(next_op_id, -1)
            if path and planned_start > path[-1].t:
                goal = path[-1]
                for wait_t in range(path[-1].t + 1, planned_start + 1):
                    path.append(TimedPose(
                        t=wait_t, x=goal.x, y=goal.y,
                        action="WAIT", operation_id=None,
                    ))

            # 预约路径
            if not self.space_time_astar.reserve_path(
                path, rid, operation.machine_id
            ):
                self._log_event("reservation_failed",
                    f"{rid}: path reservation failed")
                self.state_machine.unlock_machine(operation.machine_id)
                robot.status = RobotStatus.WAITING_CONFLICT
                continue

            # *** 提前预约工作区 ***
            # 规划时立即预约未来的工作时间，让其他机器人的
            # SpaceTimeA* 能预见并自然绕行。
            work_cells = self.footprint.cells_at(goal_anchor)
            for dt in range(operation.duration):
                t = path[-1].t + dt
                self.reservation_table.pose_cells_by_time[t][work_cells] = rid

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

        # *** 清理临时注入的静止机器人预约 ***
        self.reservation_table.release_by_prefix("_STATIC_")

    def _invalidate_conflicting_paths(self, reserved_rid: str) -> None:
        """检查新预约是否使其他机器人的已规划路径失效。"""
        for other_rid, other_robot in self.robots.items():
            if other_rid == reserved_rid or other_robot.finished:
                continue
            if other_robot.current_path is None:
                continue
            for i in range(other_robot.path_index, len(other_robot.current_path)):
                pose = other_robot.current_path[i]
                cells = self.footprint.cells_at(Cell(pose.x, pose.y))
                conflicts = self.reservation_table.get_conflicts_at(pose.t, cells)
                other_conflicts = [c for c in conflicts if c != other_rid]
                if other_conflicts:
                    self._cancel_current_plan(other_rid)
                    self._log_event("conflict_detected",
                        f"{other_rid}: path invalidated by {reserved_rid} at t={pose.t}")
                    break

    def _inject_static_robots(self) -> int:
        """将静止且不会在本次规划中移动的机器人的位置注入预约表。

        注入所有当前没有有效移动路径的机器人。规划自身时，带
        _STATIC_ 前缀的自身预约会被排除；其他机器人仍能看到它。
        """
        count = 0
        for rid, robot in self.robots.items():
            if robot.current_anchor is None:
                continue
            if robot.status == RobotStatus.MOVING and robot.current_path is not None:
                continue
            cells = self.footprint.cells_at(robot.current_anchor)
            # 滚动占用窗口：足以让规划器先等待/绕行；窗口之后仍由
            # 执行前安全仲裁兜底，避免把暂时静止者当作永久墙体。
            horizon = min(self._max_steps + 1, self.current_time + 16)
            self.reservation_table.reserve_pose_range(
                self.current_time, horizon, cells, f"_STATIC_{rid}"
            )
            count += horizon - self.current_time
        return count

    # ==================================================================
    # 内部方法：返回起点
    # ==================================================================

    def _plan_two_legs(self, start: Cell, waypoint: Cell, goal: Cell,
                       start_time: int, rid: str) -> list[TimedPose] | None:
        """两段规划: start → waypoint(主干道) → goal。"""
        horizon = min(self._max_steps, start_time + self._planning_window)
        leg1 = self.space_time_astar.plan(start, waypoint, start_time,
                                          horizon, robot_id=rid)
        if leg1 is None:
            return None
        leg2 = self.space_time_astar.plan(waypoint, goal, leg1[-1].t,
                                          horizon, robot_id=rid)
        if leg2 is None:
            return None
        return leg1 + leg2[1:]  # 去掉 leg2 的 START

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
            max_time=min(self._max_steps, self.current_time + self._planning_window),
            robot_id=rid,
        )

        if path is None:
            self._log_event("return_to_start",
                f"{rid}: cannot plan return path, will retry")
            return

        if not self.space_time_astar.reserve_path(path, rid):
            self._log_event("return_to_start",
                f"{rid}: return path reservation failed, will retry")
            return

        robot.current_path = path
        robot.path_index = 0
        robot.current_op_id = "__RETURN__"
        robot.status = RobotStatus.MOVING
        self._log_event("return_to_start",
            f"{rid}: returning to start ({start_anchor.x},{start_anchor.y}), "
            f"path={len(path)} steps, arrival_t={path[-1].t}")

    def _plan_yield_to_home(self, rid: str, robot: RobotRuntime) -> bool:
        """为低优先级机器人规划到其第28行停车位的主动让行路径。"""
        home = robot.spec.start_anchor
        if robot.current_anchor == home:
            robot.yield_requested = False
            return False
        path = self.space_time_astar.plan(
            robot.current_anchor,
            home,
            self.current_time,
            min(self._max_steps, self.current_time + self._planning_window),
            robot_id=rid,
        )
        if path is None or not self.space_time_astar.reserve_path(path, rid):
            robot.status = RobotStatus.WAITING_CONFLICT
            robot.retry_after = self.current_time + 8
            return True
        robot.current_path = path
        robot.path_index = 0
        robot.current_op_id = "__YIELD__"
        robot.status = RobotStatus.MOVING
        self._log_event("yield_planned",
            f"{rid}: yielding to parking position ({home.x},{home.y})")
        return True

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
                if self.current_time < robot.retry_after:
                    actions[rid] = {"action_type": "wait", "reason": "retry_backoff"}
                    continue
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

                # 路径可能已被其他机器人无效化
                if robot.current_path is None:
                    continue

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
                    elif robot.current_op_id == "__YIELD__":
                        robot.yield_requested = False
                        robot.status = RobotStatus.IDLE
                        robot.current_op_id = None
                        robot.current_path = None
                        self._log_event("yield_complete",
                            f"{rid}: reached parking position; task will resume")
                    # 作业开始：预约整个作业期间的本体占用
                    elif robot.current_op_id:
                        op = self.operations.get(robot.current_op_id)
                        if op:
                            robot.work_remaining = op.duration
                            robot.status = RobotStatus.WORKING
                            # 立即预约工作期间完整 2x4 本体
                            work_cells = self.footprint.cells_at(
                                Cell(pose.x, pose.y)
                            )
                            for dt in range(op.duration):
                                self.reservation_table.reserve_pose(
                                    pose.t + dt, work_cells, rid
                                )
                            # 检查其他机器人的路径是否因此失效
                            self._invalidate_conflicting_paths(rid)
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

    def _filter_unsafe_actions(self, actions: dict[str, dict]) -> dict[str, dict]:
        """执行前用实时位置做最后一道安全仲裁。

        预约路径生成后，另一台机器人可能因前置条件变成静止障碍。
        此处禁止任何动作的扫掠区域穿过其他机器人的当前 footprint，
        同时禁止本时间步内两个已接受动作的扫掠区域相交。
        """
        safe = dict(actions)
        current_cells = {
            rid: self.footprint.cells_at(robot.current_anchor)
            for rid, robot in self.robots.items()
            if robot.current_anchor is not None and not robot.finished
        }
        move_ids = sorted(
            rid for rid, action in actions.items()
            if action.get("action_type") == "move"
        )
        sweeps: dict[str, frozenset[Cell]] = {}
        for rid in move_ids:
            pose = actions[rid]["pose"]
            sweeps[rid] = FootprintValidator.swept_cells(
                self.robots[rid].current_anchor, Cell(pose.x, pose.y), self.footprint
            )

        cancelled: dict[str, str] = {}
        stationary_ids = set(current_cells) - set(move_ids)
        for rid in move_ids:
            blocker = next((
                other for other in stationary_ids if sweeps[rid] & current_cells[other]
            ), None)
            if blocker:
                cancelled[rid] = blocker

        # 同步移动冲突采用稳定优先级：ID 字典序较小者先行，另一台等待。
        for i, rid in enumerate(move_ids):
            if rid in cancelled:
                continue
            for other in move_ids[i + 1:]:
                if other not in cancelled and sweeps[rid] & sweeps[other]:
                    # 两条旧路径已无法按原时间表安全执行；双方都停下并
                    # 从实时位置重新规划，避免一方等待后路径整体错位。
                    cancelled[rid] = other
                    cancelled[other] = rid
                    break

        for rid, blocker in cancelled.items():
            safe[rid] = {"action_type": "wait", "reason": "safety_guard"}
            self._cancel_current_plan(rid)
            self._log_event(
                "collision_avoided",
                f"{rid}: movement held at {self.robots[rid].current_anchor}; "
                f"blocked_by={blocker}",
            )

        # 多机器人按需让行：如果一个移动机器人被静止机器人本体挡住，
        # 且挡路者当前没有作业/移动，则让挡路者主动驶回停车位。这样
        # 避免“每项操作后都回停车位”的过度保守，也避免在窄通道中
        # 无限重规划撞上同一个静止 footprint。
        if len(self.robots) > 2:
            for blocker in sorted(set(cancelled.values())):
                blocking_robot = self.robots.get(blocker)
                if blocking_robot is None or blocking_robot.finished:
                    continue
                if blocking_robot.status in (RobotStatus.WORKING, RobotStatus.MOVING):
                    continue
                if blocking_robot.current_anchor == blocking_robot.spec.start_anchor:
                    continue
                blocking_robot.yield_requested = True
                blocking_robot.status = RobotStatus.IDLE
                self.reservation_table.release_future(blocker, self.current_time)
                safe[blocker] = {"action_type": "wait", "reason": "yield_clearance"}
                self._log_event(
                    "yield_requested",
                    f"{blocker}: clearing blocked passage for other robot",
                )

        # A 执行任务优先。发生 A/B 冲突时，B 在退避结束后主动驶向
        # 自己的安全停车位，而不是留在冲突点反复等待。
        if (
            cancelled and len(self.robots) == 2
            and "A_1" in self.robots and "B_1" in self.robots
        ):
            b = self.robots["B_1"]
            if not b.finished and b.status != RobotStatus.WORKING:
                b.yield_requested = True
                if "B_1" not in cancelled:
                    self._cancel_current_plan("B_1")
                    safe["B_1"] = {"action_type": "wait", "reason": "yield_to_A"}
                self._log_event("yield_requested", "B_1: yielding right-of-way to A_1")

        return safe

    def _cancel_current_plan(self, rid: str) -> None:
        """安全地撤销未完成路径，并把任务放回队首等待重规划。"""
        robot = self.robots[rid]
        self.reservation_table.release_future(rid, self.current_time)
        op_id = robot.current_op_id
        if op_id and op_id != "__RETURN__" and op_id in self.operations:
            if op_id not in robot.assigned_ops:
                robot.assigned_ops.insert(0, op_id)
            self.state_machine.unlock_machine(self.operations[op_id].machine_id)
        robot.current_op_id = None
        robot.current_path = None
        robot.path_index = 0
        robot.status = RobotStatus.WAITING_CONFLICT
        robot.retry_after = self.current_time + 8

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
                # 不在每项操作后强制回停车区。静止/作业中的机器人会被
                # 注入预约表，后续路径规划可以等待或绕行；强制停车会
                # 造成 2A1B/4A2B 大量无意义移动，并破坏 CP-SAT 流水线并行性。

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
