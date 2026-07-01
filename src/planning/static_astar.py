"""Footprint-aware A* 路径规划 v4.0

基于有效姿态图的静态 A* 路径规划器。

用途：
1. 预计算服务点之间的最短旅行时间矩阵
2. 为 Space-Time A* 提供启发函数
3. 检查可达性
"""

from __future__ import annotations

import heapq
from collections import defaultdict

from ..domain.models import Cell
from ..map.pose_graph import PoseGraph


class StaticAStar:
    """基于有效姿态图的静态 A* 路径规划器。"""

    def __init__(self, pose_graph: PoseGraph) -> None:
        """初始化规划器。

        Args:
            pose_graph: 预先构建的有效姿态图
        """
        self._graph = pose_graph
        # 缓存：(start, goal) -> (path, cost)
        self._path_cache: dict[tuple[Cell, Cell], tuple[list[Cell], int]] = {}
        # 缓存：旅行矩阵
        self._travel_matrix: dict[tuple[str, str], int] = {}

    # ------------------------------------------------------------------
    @staticmethod
    def _heuristic(a: Cell, b: Cell) -> int:
        """Manhattan 距离启发函数。

        Args:
            a, b: 两个锚点

        Returns:
            Manhattan 距离
        """
        return abs(a.x - b.x) + abs(a.y - b.y)

    # ------------------------------------------------------------------
    def plan(self, start: Cell, goal: Cell) -> tuple[list[Cell], int] | None:
        """标准 A* 搜索，返回最短路径及代价。

        Args:
            start: 起始锚点
            goal: 目标锚点

        Returns:
            (path, cost) 或 None（不可达）
            path 包含 start 和 goal
        """
        # 检查缓存
        cache_key = (start, goal)
        if cache_key in self._path_cache:
            return self._path_cache[cache_key]

        # 起点/终点有效性
        if not self._graph.is_valid_pose(start):
            return None
        if not self._graph.is_valid_pose(goal):
            return None

        # A* 搜索
        open_set: list[tuple[int, int, Cell]] = []  # (f, tiebreaker, node)
        heapq.heappush(open_set, (self._heuristic(start, goal), 0, start))

        g_score: dict[Cell, int] = {start: 0}
        came_from: dict[Cell, Cell] = {}
        tiebreaker = 1

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current == goal:
                # 重建路径
                path = self._reconstruct_path(came_from, current)
                result = (path, g_score[current])
                self._path_cache[cache_key] = result
                return result

            for neighbor, cost in self._graph.get_neighbors(current):
                tentative_g = g_score[current] + cost
                if tentative_g < g_score.get(neighbor, float("inf")):
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self._heuristic(neighbor, goal)
                    tiebreaker += 1
                    heapq.heappush(open_set, (f_score, tiebreaker, neighbor))
                    came_from[neighbor] = current

        # 不可达
        return None

    # ------------------------------------------------------------------
    def plan_multi_goal(
        self, start: Cell, goals: set[Cell]
    ) -> tuple[list[Cell], Cell, int] | None:
        """到多个目标中最近的那个。

        搜索在到达第一个目标时停止。

        Args:
            start: 起始锚点
            goals: 目标锚点集合

        Returns:
            (path, reached_goal, cost) 或 None（全部不可达）
        """
        if not self._graph.is_valid_pose(start):
            return None
        if not goals:
            return None

        # A* 多目标
        open_set: list[tuple[int, int, Cell]] = []
        min_h = min(self._heuristic(start, g) for g in goals)
        heapq.heappush(open_set, (min_h, 0, start))

        g_score: dict[Cell, int] = {start: 0}
        came_from: dict[Cell, Cell] = {}
        tiebreaker = 1

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current in goals:
                path = self._reconstruct_path(came_from, current)
                return path, current, g_score[current]

            for neighbor, cost in self._graph.get_neighbors(current):
                tentative_g = g_score[current] + cost
                if tentative_g < g_score.get(neighbor, float("inf")):
                    g_score[neighbor] = tentative_g
                    h = min(self._heuristic(neighbor, g) for g in goals)
                    f_score = tentative_g + h
                    tiebreaker += 1
                    heapq.heappush(open_set, (f_score, tiebreaker, neighbor))
                    came_from[neighbor] = current

        return None

    # ------------------------------------------------------------------
    def precompute_travel_matrix(
        self,
        sources: dict[str, Cell],
        targets: dict[str, Cell],
    ) -> dict[tuple[str, str], int]:
        """预计算所有 source -> target 的最短距离。

        同时缓存路径。

        Args:
            sources: {src_id: anchor} 起点字典
            targets: {tgt_id: anchor} 终点字典

        Returns:
            {(src_id, tgt_id): distance} 距离矩阵
        """
        matrix: dict[tuple[str, str], int] = {}

        for src_id, src_cell in sources.items():
            for tgt_id, tgt_cell in targets.items():
                result = self.plan(src_cell, tgt_cell)
                if result is not None:
                    _, cost = result
                    matrix[(src_id, tgt_id)] = cost
                else:
                    matrix[(src_id, tgt_id)] = -1  # 不可达标记

        self._travel_matrix.update(matrix)
        return matrix

    # ------------------------------------------------------------------
    @staticmethod
    def _reconstruct_path(
        came_from: dict[Cell, Cell], current: Cell
    ) -> list[Cell]:
        """从 came_from 字典重建路径。

        Args:
            came_from: 前驱节点映射
            current: 终点

        Returns:
            从起点到终点的完整路径（含起点和终点）
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
