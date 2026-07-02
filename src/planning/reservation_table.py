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
        swept_cells_by_interval: dict[int, dict[frozenset[Cell], str]]
            开始时间 t_start -> {扫掠格集合: 机器人ID}
            (swept interval 的结束时间通过 swept_ends 记录)
        swept_ends: dict[int, int]
            开始时间 -> 结束时间
        service_locks: dict[str, list[tuple[int, int, str]]]
            机器ID -> [(t_start, t_end, robot_id), ...]
    """

    def __init__(self) -> None:
        """初始化空的预约表。"""
        self.pose_cells_by_time: dict[int, dict[frozenset[Cell], str]] = defaultdict(dict)
        self.swept_cells_by_interval: dict[int, dict[frozenset[Cell], str]] = defaultdict(dict)
        self.swept_ends: dict[int, int] = {}
        self.service_locks: dict[str, list[tuple[int, int, str]]] = defaultdict(list)

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
        self.swept_cells_by_interval[t_start][cells] = robot_id
        self.swept_ends[t_start] = t_end
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

    def is_pose_free(
        self,
        t: int,
        cells: frozenset[Cell],
        exclude_robot: str | None = None,
    ) -> bool:
        """检查时间t的pose占用是否空闲。

        Args:
            t: 时间步
            cells: 要检查的格子集合
            exclude_robot: 排除的机器人ID（自己的预约不算冲突）

        Returns:
            True 如果空闲
        """
        for occupied_cells, robot_id in self.pose_cells_by_time.get(t, {}).items():
            if exclude_robot and robot_id == exclude_robot:
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
        for s_start, occupied_cells_dict in self.swept_cells_by_interval.items():
            s_end = self.swept_ends.get(s_start)
            if s_end is None:
                continue
            # 检查区间是否重叠
            if t_start >= s_end or t_end <= s_start:
                continue
            for occ_cells, robot_id in occupied_cells_dict.items():
                if exclude_robot and robot_id == exclude_robot:
                    continue
                if cells & occ_cells:
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
            if exclude_robot and robot_id == exclude_robot:
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
            if exclude_robot and robot_id == exclude_robot:
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
        for s_start, occupied_cells_dict in self.swept_cells_by_interval.items():
            s_end = self.swept_ends.get(s_start)
            if s_end is None:
                continue
            if t_start >= s_end or t_end <= s_start:
                continue
            for occ_cells, robot_id in occupied_cells_dict.items():
                if exclude_robot and robot_id == exclude_robot:
                    continue
                if robot_id in conflicts:
                    continue
                if cells & occ_cells:
                    conflicts.append(robot_id)
        return conflicts

    # ==================================================================
    # 临时预约管理（用于注入静止机器人位置）
    # ==================================================================

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
        for t_start in self.swept_cells_by_interval:
            if t_start >= from_time:
                cells_to_remove: list[frozenset[Cell]] = []
                for cells, rid in self.swept_cells_by_interval[t_start].items():
                    if rid == robot_id:
                        cells_to_remove.append(cells)
                for cells in cells_to_remove:
                    del self.swept_cells_by_interval[t_start][cells]
                if not self.swept_cells_by_interval[t_start]:
                    swept_to_remove.append(t_start)

        for t_start in swept_to_remove:
            del self.swept_cells_by_interval[t_start]
            self.swept_ends.pop(t_start, None)

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
        self.swept_ends.clear()
        self.service_locks.clear()

    # ==================================================================
    # 统计
    # ==================================================================

    def pose_count(self) -> int:
        """返回pose预约总数。"""
        return sum(len(d) for d in self.pose_cells_by_time.values())

    def swept_count(self) -> int:
        """返回swept预约总数。"""
        return sum(len(d) for d in self.swept_cells_by_interval.values())
