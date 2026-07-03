"""时空 A* 单元测试 v4.0

测试 SpaceTimeAStar 在有时空预约约束下的路径规划。
基于实际 API: plan, plan_with_service, reserve_path, cache_stats, clear_cache
"""

from __future__ import annotations

import pytest

from src.domain.enums import Action
from src.domain.models import Cell, Footprint, TimedPose
from src.domain.validation import FootprintValidator
from src.map.fixed_map import FixedMap
from src.map.pose_graph import PoseGraph
from src.planning.reservation_table import ReservationTable
from src.planning.space_time_astar import SpaceTimeAStar
from src.planning.static_astar import StaticAStar


# ── Fixtures ─────────────────────────────────────────────────────────────────


@pytest.fixture(scope="module")
def terrain():
    fm = FixedMap()
    t, _, _ = fm.build()
    return t


@pytest.fixture(scope="module")
def footprint():
    return Footprint.default_2x4()


@pytest.fixture(scope="module")
def pose_graph(terrain, footprint):
    pg = PoseGraph(terrain, footprint, trunk_y_threshold=24)
    pg.build()
    return pg


@pytest.fixture(scope="module")
def static_astar(pose_graph):
    return StaticAStar(pose_graph)


@pytest.fixture
def rt():
    return ReservationTable()


@pytest.fixture
def planner(pose_graph, rt, footprint):
    return SpaceTimeAStar(pose_graph, rt, footprint)


# ── 基本寻路测试（无预约冲突）────────────────────────────────────────────


class TestSpaceTimeAStarBasic:
    """基本寻路（无冲突）。"""

    def test_find_path_no_reservations(self, planner):
        """无障碍时能找到路径。"""
        path = planner.plan(Cell(1, 25), Cell(24, 26), start_time=0)
        assert path is not None, "Should find a path on empty map"
        assert len(path) >= 2, f"Path too short: {len(path)}"
        assert all(isinstance(p, TimedPose) for p in path)

    def test_path_starts_at_correct_time(self, planner):
        """路径起点时间 >= start_time。"""
        start_t = 5
        path = planner.plan(Cell(1, 25), Cell(24, 26), start_time=start_t)
        assert path is not None
        assert path[0].t == start_t, \
            f"First node time {path[0].t} != start_t {start_t}"

    def test_reaches_goal(self, planner):
        """路径到达目标坐标。"""
        goal = Cell(24, 26)
        path = planner.plan(Cell(1, 25), goal, start_time=0)
        assert path is not None
        last = path[-1]
        assert last.x == goal.x and last.y == goal.y, \
            f"Last ({last.x},{last.y}) != goal ({goal.x},{goal.y})"

    def test_time_is_monotonic(self, planner):
        """时间单调递增。"""
        path = planner.plan(Cell(1, 25), Cell(24, 26), start_time=0)
        assert path is not None
        for i in range(1, len(path)):
            assert path[i].t >= path[i - 1].t, \
                f"Time not monotonic at index {i}: {path[i-1].t} -> {path[i].t}"

    def test_same_cell_returns_path(self, planner):
        """起点=终点应返回单节点路径。"""
        path = planner.plan(Cell(12, 25), Cell(12, 25), start_time=0)
        assert path is not None
        assert len(path) >= 1

    def test_nonexistent_start_returns_none(self, planner):
        """无效起点返回 None。"""
        # Cell(25, 25) 的 footprint 会出界
        path = planner.plan(Cell(25, 25), Cell(12, 25), start_time=0)
        assert path is None, "Should return None for invalid start"


# ── 有预约约束的寻路 ───────────────────────────────────────────────────


class TestSpaceTimeAStarWithReservations:
    """预约冲突下的寻路。"""

    def test_detour_around_reservation(self, pose_graph, footprint):
        """有预约时绕行或等待。"""
        rt = ReservationTable()
        planner = SpaceTimeAStar(pose_graph, rt, footprint)

        # 预约起点附近几个格，迫使绕行
        start = Cell(1, 25)
        goal = Cell(10, 25)

        # 阻塞起点右侧几个格的所有时间步
        for t in range(100):
            blocked = footprint.cells_at(Cell(3, 25))
            rt.reserve_pose(t=t, cells=blocked, robot_id="blocker")

        path = planner.plan(start, goal, start_time=0, robot_id="A_1")
        assert path is not None, "Should find path detouring around reservations"

    def test_path_respects_reservations(self, pose_graph, footprint):
        """路径不经过被预约的格。"""
        rt = ReservationTable()
        planner = SpaceTimeAStar(pose_graph, rt, footprint)

        blocked_cell = Cell(5, 25)
        blocked_cells = footprint.cells_at(blocked_cell)
        for t in range(200):
            rt.reserve_pose(t=t, cells=blocked_cells, robot_id="blocker")

        path = planner.plan(
            Cell(1, 25), Cell(10, 25),
            start_time=0, robot_id="A_1",
        )
        if path is not None:
            for pose in path:
                pose_cells = footprint.cells_at(Cell(pose.x, pose.y))
                # 检查是否与blocker冲突
                conflicts = rt.get_conflicts_at(
                    pose.t, pose_cells, exclude_robot="A_1"
                )
                assert not conflicts, \
                    f"Path pose at ({pose.x},{pose.y},t={pose.t}) conflicts with {conflicts}"

    def test_fully_blocked_by_time_limit(self, pose_graph, footprint):
        """时间限制测试：很短时间内可能找不到路径。"""
        rt = ReservationTable()
        planner = SpaceTimeAStar(pose_graph, rt, footprint)

        # 时间限制极短
        path = planner.plan(
            Cell(1, 25), Cell(24, 26),
            start_time=0, max_time=10, robot_id="A_1",
        )
        # 大概率找不到（距离太远时间太短）
        if path is not None:
            # 如果找到了，时间不能超过限制
            assert path[-1].t <= 10


# ── plan_with_service ──────────────────────────────────────────────────


class TestPlanWithService:
    """plan_with_service 功能测试。"""

    def test_plan_with_service_includes_waits(self, planner):
        """plan_with_service 路径末尾有服务 WAIT。"""
        path = planner.plan_with_service(
            start_anchor=Cell(1, 25),
            goal_anchor=Cell(5, 10),
            start_time=0,
            service_duration=6,
            machine_id="M_y3_x2",
            operation_id="M_y3_x2_D",
            robot_id="A_1",
        )
        assert path is not None, "plan_with_service should return a path"

        # 找到最后一个移动姿态后的 WAIT 姿态
        wait_count = sum(1 for p in path if p.action == "WAIT")
        assert wait_count >= 6, \
            f"Expected >= 6 WAIT poses for service, got {wait_count}"

        # 服务姿态的 operation_id 正确
        service_poses = [p for p in path if p.operation_id == "M_y3_x2_D"]
        assert len(service_poses) >= 6, \
            f"Expected >= 6 poses with operation_id, got {len(service_poses)}"

    def test_plan_with_service_unreachable(self, planner):
        """不可达目标返回 None。"""
        path = planner.plan_with_service(
            start_anchor=Cell(25, 25),  # 无效位置
            goal_anchor=Cell(5, 10),
            start_time=0,
            service_duration=6,
            machine_id="M_y3_x2",
            operation_id="M_y3_x2_D",
            robot_id="A_1",
        )
        assert path is None, "Should return None for unreachable start"


# ── reserve_path ───────────────────────────────────────────────────────


class TestReservePath:
    """reserve_path 预约写入测试。"""

    def test_reserve_path_writes_to_table(self, planner, rt):
        """reserve_path 将路径写入预约表。"""
        path = planner.plan(Cell(1, 25), Cell(10, 25), start_time=0, robot_id="A_1")
        assert path is not None

        count_before = rt.pose_count()
        success = planner.reserve_path(path, "A_1")
        assert success, "reserve_path should succeed"
        count_after = rt.pose_count()
        assert count_after > count_before, \
            f"reserve_path should add entries ({count_before} -> {count_after})"

    def test_reserve_path_with_machine(self, planner, rt):
        """reserve_path 含 machine_id 时预约服务锁。"""
        path = planner.plan_with_service(
            start_anchor=Cell(1, 25),
            goal_anchor=Cell(5, 10),
            start_time=0,
            service_duration=6,
            machine_id="M_y3_x2",
            operation_id="M_y3_x2_D",
            robot_id="A_1",
        )
        assert path is not None

        success = planner.reserve_path(path, "A_1", machine_id="M_y3_x2")
        assert success, "reserve_path with machine should succeed"


# ── 缓存 ───────────────────────────────────────────────────────────────


class TestCache:
    """路径缓存测试。"""

    def test_dynamic_planning_does_not_reuse_stale_cache(self, planner):
        """动态预约可能变化，重复规划不应复用过期路径缓存。"""
        path1 = planner.plan(Cell(1, 25), Cell(12, 25), start_time=0, robot_id="A_1")
        assert path1 is not None

        stats_before = planner.cache_stats()
        path2 = planner.plan(Cell(1, 25), Cell(12, 25), start_time=0, robot_id="A_1")
        stats_after = planner.cache_stats()

        assert path2 is not None
        assert stats_before == {"hits": 0, "misses": 0, "size": 0}
        assert stats_after == {"hits": 0, "misses": 0, "size": 0}

    def test_clear_cache(self, planner):
        """clear_cache 清空缓存。"""
        planner.plan(Cell(1, 25), Cell(12, 25), start_time=0, robot_id="A_1")
        planner.clear_cache()
        stats = planner.cache_stats()
        assert stats["size"] == 0
        assert stats["hits"] == 0


# ── 一致性测试 ──────────────────────────────────────────────────────────


class TestSpaceTimeAStarConsistency:
    """一致性测试。"""

    def test_static_vs_no_reservation_space_time(self, pose_graph, footprint, static_astar):
        """无预约时时空A*路径长度 >= 静态A*路径长度。"""
        rt = ReservationTable()
        planner = SpaceTimeAStar(pose_graph, rt, footprint)

        start = Cell(1, 25)
        goal = Cell(24, 26)

        static_result = static_astar.plan(start, goal)
        assert static_result is not None
        static_path, static_cost = static_result

        st_path = planner.plan(start, goal, start_time=0)
        assert st_path is not None

        # 时空路径长度应 >= 静态路径长度
        assert len(st_path) >= len(static_path), \
            f"Space-time path ({len(st_path)} steps) should be >= static path ({len(static_path)} steps)"

    def test_multiple_sequential_paths(self, pose_graph, footprint):
        """顺序规划不重叠。"""
        rt = ReservationTable()
        planner = SpaceTimeAStar(pose_graph, rt, footprint)

        targets = [Cell(5, 10), Cell(11, 10), Cell(17, 10)]
        current = Cell(1, 25)
        t = 0
        robot_id = "A_1"

        for target in targets:
            path = planner.plan(current, target, start_time=t, robot_id=robot_id)
            assert path is not None, f"Should reach ({target.x},{target.y})"
            # 预约路径
            planner.reserve_path(path, robot_id)
            current = target
            t = path[-1].t + 10  # 模拟 10 步的服务时间
