"""时空预约表 v4.0

多机器人碰撞避免的核心数据结构。

记录三类预约:
1. pose_cells_by_time: 时间t某机器人本体的2x4占用格
2. swept_cells_by_interval: 移动区间[start, end)的扫掠占用格
3. service_locks: 机器占用区间(作业期间禁止其他机器人进入该机器的区域)

预约区间是左闭右开: [t_start, t_end)
"""

from __future__ import annotations

from collections import defaultdict

from ..domain.models import Cell


class ReservationTable:
    """时空预约表 —— 多机器人碰撞避免的核心。

    所有预约区间均为左闭右开: [t_start, t_end)。

    Attributes:
        pose_cells_by_time: dict[int, dict[frozenset[Cell], str]]
            时间 t -> {本体占用格集合: 机器人ID}
        swept_cells_by_interval: dict[int, list[tuple[int, frozenset[Cell], str]]]
            开始时间 -> [(结束时间, 扫掠格集合, 机器人ID), ...]
        service_locks: dict[str, list[tuple[int, int, str]]]
            机器ID -> [(t_start, t_end, robot_id), ...]
    """

    def __init__(self) -> None:
        """初始化空的预约表。"""
        self.pose_cells_by_time: dict[int, dict[frozenset[Cell], str]] = defaultdict(dict)
        self.swept_cells_by_interval: dict[
            int, list[tuple[int, frozenset[Cell], str]]
        ] = defaultdict(list)
        self.service_locks: dict[str, list[tuple[int, int, str]]] = defaultdict(list)
        # 当前静止机器人的持续占用，不按时间展开，避免制造海量预约。
        self.static_occupancies: dict[str, frozenset[Cell]] = {}

    # ==================================================================
    # 预约方法
    # ==================================================================

    def reserve_pose(
        self, t: int, cells: frozenset[Cell], robot_id: str
    ) -> bool:
        """在时间t预约本体占用。

        检查该时间是否已有其他机器人占用这些格。

        Args:
            t: 时间步
            cells: 本体占用的格子集合
            robot_id: 机器人ID

        Returns:
            True 如果预约成功，False 如果有冲突
        """
        if not self.is_pose_free(t, cells, exclude_robot=robot_id):
            return False
        self.pose_cells_by_time[t][cells] = robot_id
        return True

    def reserve_swept(
        self,
        t_start: int,
        t_end: int,
        cells: frozenset[Cell],
        robot_id: str,
    ) -> bool:
        """预约扫掠区域 [t_start, t_end)。

        扫掠区域 = 起点footprint ∪ 终点footprint，在转移过程中的[t_start, t_end)
        区间内被占用，禁止其他机器人在此区间进入冲突格。

        Args:
            t_start: 开始时间（含）
            t_end: 结束时间（不含）
            cells: 扫掠格子集合
            robot_id: 机器人ID

        Returns:
            True 如果预约成功
        """
        if not self.is_swept_free(t_start, t_end, cells, exclude_robot=robot_id):
            return False
        self.swept_cells_by_interval[t_start].append((t_end, cells, robot_id))
        return True

    def reserve_service(
        self,
        machine_id: str,
        t_start: int,
        t_end: int,
        robot_id: str,
    ) -> bool:
        """预约机器占用区间 [t_start, t_end)。

        作业期间禁止其他机器人进入该机器的服务区域。

        Args:
            machine_id: 离心机ID
            t_start: 开始时间（含）
            t_end: 结束时间（不含）
            robot_id: 机器人ID

        Returns:
            True 如果预约成功，False 如果冲突
        """
        if not self.is_service_free(machine_id, t_start, t_end, exclude_robot=robot_id):
            return False
        self.service_locks[machine_id].append((t_start, t_end, robot_id))
        return True

    # ==================================================================
    # 查询方法
    # ==================================================================

    @staticmethod
    def _is_self(exclude_robot: str | None, robot_id: str) -> bool:
        """检查 robot_id 是否属于 exclude_robot 自身（含 _STATIC_ 临时预约）。"""
        if exclude_robot is None:
            return False
        return robot_id == exclude_robot or robot_id == f"_STATIC_{exclude_robot}"

    def is_pose_free(
        self,
        t: int,
        cells: frozenset[Cell],
        exclude_robot: str | None = None,
    ) -> bool:
        """检查时间t的pose占用是否空闲。"""
        for robot_id, occupied_cells in self.static_occupancies.items():
            if not self._is_self(exclude_robot, robot_id) and cells & occupied_cells:
                return False
        for occupied_cells, robot_id in self.pose_cells_by_time.get(t, {}).items():
            if self._is_self(exclude_robot, robot_id):
                continue
            if cells & occupied_cells:
                return False
        return True

    def is_swept_free(
        self,
        t_start: int,
        t_end: int,
        cells: frozenset[Cell],
        exclude_robot: str | None = None,
    ) -> bool:
        """检查扫掠区域在 [t_start, t_end) 是否与其他预约冲突。

        Args:
            t_start: 开始时间（含）
            t_end: 结束时间（不含）
            cells: 要检查的格子集合
            exclude_robot: 排除的机器人ID

        Returns:
            True 如果空闲
        """
        for s_start, reservations in self.swept_cells_by_interval.items():
            for s_end, occ_cells, robot_id in reservations:
                if t_start >= s_end or t_end <= s_start:
                    continue
                if self._is_self(exclude_robot, robot_id):
                    continue
                if cells & occ_cells:
                    return False
        return True

    def is_transition_free(
        self,
        t_start: int,
        t_end: int,
        swept_cells: frozenset[Cell],
        destination_cells: frozenset[Cell],
        exclude_robot: str | None = None,
    ) -> bool:
        """检查移动与姿态、扫掠两类预约均不冲突。"""
        if not self.is_pose_free(t_end, destination_cells, exclude_robot):
            return False
        if not self.is_swept_free(t_start, t_end, swept_cells, exclude_robot):
            return False
        # 防止移动机器人在动作区间穿过静止机器人。
        for t in range(t_start, t_end + 1):
            if not self.is_pose_free(t, swept_cells, exclude_robot):
                return False
        return True

    def is_service_free(
        self,
        machine_id: str,
        t_start: int,
        t_end: int,
        exclude_robot: str | None = None,
    ) -> bool:
        """检查机器在 [t_start, t_end) 是否已被预约服务。

        Args:
            machine_id: 离心机ID
            t_start: 开始时间（含）
            t_end: 结束时间（不含）
            exclude_robot: 排除的机器人ID

        Returns:
            True 如果空闲
        """
        for lock_start, lock_end, robot_id in self.service_locks.get(machine_id, []):
            if self._is_self(exclude_robot, robot_id):
                continue
            if t_start < lock_end and t_end > lock_start:
                return False
        return True

    # ==================================================================
    # 冲突检测
    # ==================================================================

    def get_conflicts_at(
        self,
        t: int,
        cells: frozenset[Cell],
        exclude_robot: str | None = None,
    ) -> list[str]:
        """返回在时间t与给定cells冲突的机器人ID列表。

        Args:
            t: 时间步
            cells: 要检查的格子集合
            exclude_robot: 排除的机器人ID

        Returns:
            冲突机器人ID列表
        """
        conflicts: list[str] = []
        for occupied_cells, robot_id in self.pose_cells_by_time.get(t, {}).items():
            if self._is_self(exclude_robot, robot_id):
                continue
            if cells & occupied_cells:
                conflicts.append(robot_id)
        return conflicts

    def get_swept_conflicts(
        self,
        t_start: int,
        t_end: int,
        cells: frozenset[Cell],
        exclude_robot: str | None = None,
    ) -> list[str]:
        """返回在区间 [t_start, t_end) 与给定cells有扫掠冲突的机器人ID列表。

        Args:
            t_start: 开始时间（含）
            t_end: 结束时间（不含）
            cells: 要检查的格子集合
            exclude_robot: 排除的机器人ID

        Returns:
            冲突机器人ID列表
        """
        conflicts: list[str] = []
        for s_start, reservations in self.swept_cells_by_interval.items():
            for s_end, occ_cells, robot_id in reservations:
                if t_start >= s_end or t_end <= s_start:
                    continue
                if self._is_self(exclude_robot, robot_id):
                    continue
                if robot_id in conflicts:
                    continue
                if cells & occ_cells:
                    conflicts.append(robot_id)
        return conflicts

    # ==================================================================
    # 临时预约管理（用于注入静止机器人位置）
    # ==================================================================

    def reserve_static(self, cells: frozenset[Cell], robot_id: str) -> None:
        """记录机器人在整个当前规划周期内持续占用的位置。"""
        self.static_occupancies[robot_id] = cells

    def reserve_pose_range(
        self,
        t_start: int,
        t_end: int,
        cells: frozenset[Cell],
        robot_id: str,
    ) -> list[tuple[int, frozenset[Cell], str]]:
        """在 [t_start, t_end) 范围内预约某机器人的占用。

        用于在规划前注入静止机器人（IDLE/WORKING/FINISHED）的位置，
        使 SpaceTimeA* 能避开它们。

        Args:
            t_start: 开始时间（含）
            t_end: 结束时间（不含）
            cells: 占用格子集合
            robot_id: 机器人ID（建议使用带前缀的标识如 "_STATIC_A_1"）

        Returns:
            成功预约的 (t, cells, robot_id) 列表，供后续释放
        """
        reserved: list[tuple[int, frozenset[Cell], str]] = []
        for t in range(t_start, t_end):
            # 使用 force 语义：不检查冲突，直接覆盖式预约
            # （临时预约在规划完成后会被清理）
            self.pose_cells_by_time[t][cells] = robot_id
            reserved.append((t, cells, robot_id))
        return reserved

    def release_by_prefix(self, prefix: str = "_STATIC_") -> int:
        """释放所有 robot_id 以指定前缀开头的预约。

        Args:
            prefix: 机器人ID前缀，默认 "_STATIC_"

        Returns:
            释放的预约条目数量
        """
        count = 0

        static_ids = [rid for rid in self.static_occupancies if rid.startswith(prefix)]
        for rid in static_ids:
            del self.static_occupancies[rid]
            count += 1

        # 释放 pose 预约
        times_to_remove: list[int] = []
        for t in self.pose_cells_by_time:
            cells_to_remove: list[frozenset[Cell]] = []
            for cells, rid in self.pose_cells_by_time[t].items():
                if rid.startswith(prefix):
                    cells_to_remove.append(cells)
                    count += 1
            for cells in cells_to_remove:
                del self.pose_cells_by_time[t][cells]
            if not self.pose_cells_by_time[t]:
                times_to_remove.append(t)
        for t in times_to_remove:
            del self.pose_cells_by_time[t]

        return count

    # ==================================================================
    # 释放与清理
    # ==================================================================

    def prune_before(self, time: int) -> None:
        """删除已经结束的历史预约，保持长期仿真的查询开销稳定。"""
        for t in [t for t in self.pose_cells_by_time if t < time]:
            del self.pose_cells_by_time[t]
        for start, reservations in list(self.swept_cells_by_interval.items()):
            remaining = [item for item in reservations if item[0] > time]
            if remaining:
                self.swept_cells_by_interval[start] = remaining
            else:
                del self.swept_cells_by_interval[start]
        for machine_id, locks in list(self.service_locks.items()):
            self.service_locks[machine_id] = [item for item in locks if item[1] > time]

    def release_future(self, robot_id: str, from_time: int) -> None:
        """释放某机器人在 from_time 及之后的所有预约。

        用于路径重新规划或机器人故障恢复。

        Args:
            robot_id: 机器人ID
            from_time: 起始时间（含）
        """
        # 释放 pose 预约
        times_to_remove: list[int] = []
        for t in self.pose_cells_by_time:
            if t >= from_time:
                cells_to_remove: list[frozenset[Cell]] = []
                for cells, rid in self.pose_cells_by_time[t].items():
                    if rid == robot_id:
                        cells_to_remove.append(cells)
                for cells in cells_to_remove:
                    del self.pose_cells_by_time[t][cells]
                if not self.pose_cells_by_time[t]:
                    times_to_remove.append(t)

        for t in times_to_remove:
            del self.pose_cells_by_time[t]

        # 释放 swept 预约
        swept_to_remove: list[int] = []
        for t_start, reservations in self.swept_cells_by_interval.items():
            if t_start >= from_time:
                self.swept_cells_by_interval[t_start] = [
                    item for item in reservations if item[2] != robot_id
                ]
                if not self.swept_cells_by_interval[t_start]:
                    swept_to_remove.append(t_start)

        for t_start in swept_to_remove:
            del self.swept_cells_by_interval[t_start]

        # 释放 service 预约
        for machine_id in self.service_locks:
            self.service_locks[machine_id] = [
                (ls, le, rid)
                for ls, le, rid in self.service_locks[machine_id]
                if rid != robot_id or ls < from_time
            ]

    def clear(self) -> None:
        """清空所有预约。"""
        self.pose_cells_by_time.clear()
        self.swept_cells_by_interval.clear()
        self.service_locks.clear()
        self.static_occupancies.clear()

    # ==================================================================
    # 统计
    # ==================================================================

    def pose_count(self) -> int:
        """返回pose预约总数。"""
        return sum(len(d) for d in self.pose_cells_by_time.values())

    def swept_count(self) -> int:
        """返回swept预约总数。"""
        return sum(len(d) for d in self.swept_cells_by_interval.values())
