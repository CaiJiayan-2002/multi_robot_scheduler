"""时空A*路径规划器 v4.0

搜索在 (x, y, t) 空间中的无碰撞路径。

搜索状态: (anchor: Cell, time: int)
邻居动作: UP/DOWN/LEFT/RIGHT/WAIT（由PoseGraph提供合法转移）

在每个扩展节点检查:
1. 目标姿态的2x4本体是否与预约冲突
2. 扫掠区域是否与预约冲突
3. 地图边界和静态障碍（由PoseGraph预处理）
4. 主干道移动规则（由PoseGraph预处理）
"""

from __future__ import annotations

import heapq
from collections import defaultdict

from ..domain.models import Cell, Footprint, TimedPose
from ..domain.validation import FootprintValidator
from ..domain.enums import Action
from ..map.pose_graph import PoseGraph
from .reservation_table import ReservationTable


class SpaceTimeAStar:
    """时空A*路径规划器。

    在时空图中搜索无碰撞路径，与ReservationTable交互以避开其他机器人的预约。

    Attributes:
        pose_graph: 有效姿态图
        reservation_table: 时空预约表
        footprint: 机器人 footprint
        validator: FootprintValidator 实例
        _path_cache: 路径缓存 {(start, goal, start_t): (path, cost)}
    """

    def __init__(
        self,
        pose_graph: PoseGraph,
        reservation_table: ReservationTable,
        footprint: Footprint,
        validator: FootprintValidator | None = None,
    ) -> None:
        """初始化时空A*规划器。

        Args:
            pose_graph: 预先构建的有效姿态图
            reservation_table: 共享的时空预约表
            footprint: 机器人 footprint
            validator: FootprintValidator 实例（可选，默认创建新实例）
        """
        self._graph = pose_graph
        self._reservations = reservation_table
        self._footprint = footprint
        self._validator = validator or FootprintValidator()

        # 路径缓存: (start, goal, start_t) -> (path, cost)
        self._path_cache: dict[tuple[Cell, Cell, int], tuple[list[TimedPose], int]] = {}
        self._cache_hits: int = 0
        self._cache_misses: int = 0

    # ==================================================================
    # 公开接口
    # ==================================================================

    def plan(
        self,
        start_anchor: Cell,
        goal_anchor: Cell,
        start_time: int,
        max_time: int = 500,
        robot_id: str | None = None,
    ) -> list[TimedPose] | None:
        """规划一条从(start, start_time)到goal的时空路径。

        在扩展每个节点时检查预约表冲突。不进行预约——
        调用者负责在路径确定后调用 reserve_path() 将路径写入预约表。

        Args:
            start_anchor: 起始锚点
            goal_anchor: 目标锚点
            start_time: 起始时间
            max_time: 时间上限（防止无限搜索）
            robot_id: 机器人ID（用于预约冲突排除自身）

        Returns:
            成功: list[TimedPose] 从start_time到到达时间的完整姿态序列
            失败: None（超时或不可达）
        """
        # 路径缓存已禁用 — 多机器人场景下，预约表随时变化，
        # 缓存路径可能在新的预约下失效，导致碰撞。
        # cache_key = (start_anchor, goal_anchor, start_time)
        # if cache_key in self._path_cache:
        #     self._cache_hits += 1
        #     return self._path_cache[cache_key][0]

        # 验证起点终点
        if not self._graph.is_valid_pose(start_anchor):
            return None
        if not self._graph.is_valid_pose(goal_anchor):
            return None

        # A* 搜索
        result = self._astar_search(
            start_anchor, goal_anchor, start_time, max_time, robot_id
        )

        if result is not None:
            return result[0]  # path
        return None

    def plan_with_service(
        self,
        start_anchor: Cell,
        goal_anchor: Cell,
        start_time: int,
        service_duration: int,
        machine_id: str,
        operation_id: str,
        max_time: int = 500,
        robot_id: str | None = None,
    ) -> list[TimedPose] | None:
        """规划到goal并在那里驻留 service_duration 时间的路径。

        路径末尾有 service_duration 个 WAIT 姿态，operation_id 标记为该操作ID。
        这些 WAIT 姿态也检查预约表冲突。

        Args:
            start_anchor: 起始锚点
            goal_anchor: 目标锚点（服务位置）
            start_time: 起始时间
            service_duration: 服务持续时间（时间步数）
            machine_id: 离心机ID
            operation_id: 操作ID
            max_time: 时间上限
            robot_id: 机器人ID

        Returns:
            成功: 完整路径（移动 + service_duration 个 WAIT 服务姿态）
            失败: None
        """
        # 1. 先规划到服务点的路径
        travel_path = self.plan(start_anchor, goal_anchor, start_time, max_time, robot_id)
        if travel_path is None:
            return None

        # 2. 到达时间
        arrival_t = travel_path[-1].t
        service_start_t = arrival_t + 1  # 服务从到达后的下一个时间步开始

        # 3. 检查服务期间目标位置是否空闲
        goal_cells = self._footprint.cells_at(goal_anchor)
        for dt in range(service_duration):
            check_t = service_start_t + dt
            if check_t > max_time:
                return None  # 超出时间限制
            if not self._reservations.is_pose_free(
                check_t, goal_cells, exclude_robot=robot_id
            ):
                return None  # 服务位置被占用

        # 4. 构建完整路径
        full_path = list(travel_path)

        # 标记最后一个移动姿态为对齐阶段
        if full_path:
            full_path[-1] = TimedPose(
                t=full_path[-1].t,
                x=full_path[-1].x,
                y=full_path[-1].y,
                action="ALIGN",
                operation_id=operation_id,
            )

        # 添加服务 WAIT 姿态
        for dt in range(service_duration):
            full_path.append(
                TimedPose(
                    t=service_start_t + dt,
                    x=goal_anchor.x,
                    y=goal_anchor.y,
                    action="WAIT",
                    operation_id=operation_id,
                )
            )

        return full_path

    # ==================================================================
    # 预约路径
    # ==================================================================

    def reserve_path(
        self, path: list[TimedPose], robot_id: str, machine_id: str | None = None
    ) -> bool:
        """将已规划的路径写入预约表。

        预约:
        1. 每个 TimedPose 的 2x4 本体占用
        2. 每个非 WAIT 转移的扫掠区域
        3. 如果提供 machine_id，预约作业期间的服务锁

        预约成功后清除路径缓存，确保其他机器人不会复用在新预约下
        已失效的旧路径。

        Args:
            path: 规划的路径
            robot_id: 机器人ID
            machine_id: 离心机ID（如果有服务操作）

        Returns:
            True 如果全部预约成功，False 如果有冲突
        """
        foot = self._footprint

        for i, pose in enumerate(path):
            cells = foot.cells_at(Cell(pose.x, pose.y))

            # 预约本体占用
            if not self._reservations.reserve_pose(pose.t, cells, robot_id):
                self._reservations.release_future(robot_id, path[0].t)
                return False

            # 预约扫掠区域（对于非WAIT动作）
            if i > 0 and pose.action != "WAIT" and pose.action != "ALIGN":
                prev = path[i - 1]
                swept = FootprintValidator.swept_cells(
                    Cell(prev.x, prev.y), Cell(pose.x, pose.y), foot
                )
                if not self._reservations.reserve_swept(
                    prev.t, pose.t + 1, swept, robot_id
                ):
                    self._reservations.release_future(robot_id, path[0].t)
                    return False

            # 预约服务锁
            if machine_id and pose.operation_id:
                # 找到作业区间的起止时间
                service_poses = [p for p in path if p.operation_id == pose.operation_id]
                if service_poses:
                    svc_start = service_poses[0].t
                    svc_end = service_poses[-1].t + 1
                    if not self._reservations.reserve_service(
                        machine_id, svc_start, svc_end, robot_id
                    ):
                        self._reservations.release_future(robot_id, path[0].t)
                        return False

        # 清除路径缓存：新预约可能使其他机器人的缓存路径失效
        self._path_cache.clear()
        return True

    # ==================================================================
    # A* 搜索实现
    # ==================================================================

    def _astar_search(
        self,
        start_anchor: Cell,
        goal_anchor: Cell,
        start_time: int,
        max_time: int,
        robot_id: str | None = None,
    ) -> tuple[list[TimedPose], int] | None:
        """时空 A* 搜索内核。"""
        foot = self._footprint
        H = self._graph.H
        W = self._graph.W

        # 开放列表: (f_score, tiebreaker, (x, y, t))
        open_set: list[tuple[int, int, tuple[int, int, int]]] = []
        start_f = self._heuristic(start_anchor, goal_anchor)
        heapq.heappush(
            open_set, (start_f, 0, (start_anchor.x, start_anchor.y, start_time))
        )

        # g_score: (x, y, t) -> int
        g_score: dict[tuple[int, int, int], int] = {
            (start_anchor.x, start_anchor.y, start_time): 0
        }

        # came_from: (x, y, t) -> (parent_x, parent_y, parent_t, action_name)
        came_from: dict[
            tuple[int, int, int], tuple[int, int, int, str]
        ] = {}

        tiebreaker = 1
        goal_cells = foot.cells_at(goal_anchor)

        while open_set:
            _, _, (cx, cy, ct) = heapq.heappop(open_set)
            current_anchor = Cell(cx, cy)

            # 到达目标
            if current_anchor == goal_anchor:
                path, cost = self._reconstruct_path(
                    came_from, cx, cy, ct, start_time, goal_anchor
                )
                return path, cost

            # 时间剪枝
            current_g = g_score[(cx, cy, ct)]
            if ct >= max_time:
                continue

            # 扩展邻居
            for neighbor_anchor, step_cost in self._graph.get_neighbors(current_anchor):
                nx, ny = neighbor_anchor.x, neighbor_anchor.y
                nt = ct + step_cost

                if nt > max_time:
                    continue

                # 判断动作类型
                action = self._validator.classify_action(current_anchor, neighbor_anchor)
                action_name = action.name if action else "MOVE"

                # 检查目标姿态预约冲突
                neighbor_cells = foot.cells_at(neighbor_anchor)
                if not self._reservations.is_pose_free(
                    nt, neighbor_cells, exclude_robot=robot_id
                ):
                    continue

                # 检查扫掠区域预约冲突（非WAIT动作）
                if action and action != Action.WAIT:
                    swept = FootprintValidator.swept_cells(
                        current_anchor, neighbor_anchor, foot
                    )
                    if not self._reservations.is_swept_free(
                        ct, nt, swept, exclude_robot=robot_id
                    ):
                        continue

                # 更新代价
                tentative_g = current_g + step_cost
                state_key = (nx, ny, nt)
                if tentative_g < g_score.get(state_key, float("inf")):
                    g_score[state_key] = tentative_g
                    f_score = tentative_g + self._heuristic(neighbor_anchor, goal_anchor)
                    tiebreaker += 1
                    heapq.heappush(open_set, (f_score, tiebreaker, state_key))
                    came_from[state_key] = (cx, cy, ct, action_name)

        return None  # 不可达

    # ==================================================================
    # 启发函数 & 路径重建
    # ==================================================================

    @staticmethod
    def _heuristic(a: Cell, b: Cell) -> int:
        """Manhattan 距离启发函数。"""
        return abs(a.x - b.x) + abs(a.y - b.y)

    def _reconstruct_path(
        self,
        came_from: dict,
        goal_x: int,
        goal_y: int,
        goal_t: int,
        start_time: int,
        goal_anchor: Cell,
    ) -> tuple[list[TimedPose], int]:
        """从 came_from 字典重建路径。"""
        poses: list[TimedPose] = []
        cx, cy, ct = goal_x, goal_y, goal_t

        # 目标节点
        poses.append(
            TimedPose(t=ct, x=cx, y=cy, action="GOAL", operation_id=None)
        )

        # 回溯
        while ct > start_time:
            state_key = (cx, cy, ct)
            if state_key not in came_from:
                break
            px, py, pt, action = came_from[state_key]
            cx, cy, ct = px, py, pt

        # 从 start 重新正向构建（保证顺序正确）
        # 实际上我们已经有正确的 ct，但为了可靠，从 start_time 开始重建
        reverse_poses: list[TimedPose] = []

        # 重新从 came_from 正向遍历
        cx, cy, ct = goal_x, goal_y, goal_t
        while ct > start_time:
            state_key = (cx, cy, ct)
            if state_key not in came_from:
                break
            reverse_poses.append(
                TimedPose(t=ct, x=cx, y=cy, action="MOVE", operation_id=None)
            )
            px, py, pt, action = came_from[state_key]
            cx, cy, ct = px, py, pt

        # 添加起始姿态
        reverse_poses.append(
            TimedPose(t=start_time, x=cx, y=cy, action="START", operation_id=None)
        )

        # 反转并修正 action 名称
        reverse_poses.reverse()
        result: list[TimedPose] = []
        for i, pose in enumerate(reverse_poses):
            if i == 0:
                action = "START"
            elif i == len(reverse_poses) - 1:
                action = "GOAL"
            else:
                # 根据前后位置判断动作
                prev = reverse_poses[i - 1]
                dx = pose.x - prev.x
                dy = pose.y - prev.y
                if dx == 0 and dy == -1:
                    action = "UP"
                elif dx == 0 and dy == 1:
                    action = "DOWN"
                elif dx == -1 and dy == 0:
                    action = "LEFT"
                elif dx == 1 and dy == 0:
                    action = "RIGHT"
                elif dx == 0 and dy == 0:
                    action = "WAIT"
                else:
                    action = "MOVE"

            result.append(
                TimedPose(
                    t=pose.t, x=pose.x, y=pose.y,
                    action=action, operation_id=None,
                )
            )

        # 最后一步是到达goal
        if result:
            result[-1] = TimedPose(
                t=result[-1].t, x=result[-1].x, y=result[-1].y,
                action="GOAL", operation_id=None,
            )

        return result, goal_t - start_time

    # ==================================================================
    # 缓存统计
    # ==================================================================

    def cache_stats(self) -> dict:
        """返回路径缓存统计。

        Returns:
            {"hits": int, "misses": int, "size": int} 字典
        """
        return {
            "hits": self._cache_hits,
            "misses": self._cache_misses,
            "size": len(self._path_cache),
        }

    def clear_cache(self) -> None:
        """清空路径缓存。"""
        self._path_cache.clear()
        self._cache_hits = 0
        self._cache_misses = 0
