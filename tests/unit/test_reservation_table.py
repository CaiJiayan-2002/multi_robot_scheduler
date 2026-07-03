"""时空预约表单元测试 v4.0

测试 ReservationTable 的预约、冲突检测、释放功能。
基于实际 API: reserve_pose, reserve_swept, reserve_service,
             is_pose_free, is_swept_free, is_service_free,
             get_conflicts_at, get_swept_conflicts,
             release_future, clear, pose_count, swept_count
"""

from __future__ import annotations

import pytest

from src.domain.models import Cell, Footprint
from src.planning.reservation_table import ReservationTable


# ── Fixtures ─────────────────────────────────────────────────────────────────


@pytest.fixture
def footprint():
    return Footprint.default_2x4()


@pytest.fixture
def rt():
    """创建空的预约表。"""
    return ReservationTable()


# ── 基本预约 (reserve_pose) ──────────────────────────────────────────────────


class TestBasicReservation:
    """pose 预约的基本功能。"""

    def test_reserve_success(self, rt):
        """预约成功返回 True。"""
        cells = frozenset([Cell(5, 10)])
        result = rt.reserve_pose(t=3, cells=cells, robot_id="A_1")
        assert result, "First reservation should succeed"

    def test_reserve_conflict(self, rt):
        """冲突预约返回 False。"""
        cells = frozenset([Cell(5, 10)])
        rt.reserve_pose(t=3, cells=cells, robot_id="A_1")
        result = rt.reserve_pose(t=3, cells=cells, robot_id="B_1")
        assert not result, "Conflicting reservation should fail"

    def test_reserve_overlapping_cells_conflict(self, rt, footprint):
        """重叠的 footprint 预约冲突。"""
        cells_a = footprint.cells_at(Cell(5, 10))  # 8 cells
        cells_b = footprint.cells_at(Cell(5, 11))  # overlaps row 1-3
        rt.reserve_pose(t=3, cells=cells_a, robot_id="A_1")
        result = rt.reserve_pose(t=3, cells=cells_b, robot_id="B_1")
        assert not result, "Overlapping footprint cells should conflict"

    def test_reserve_different_time_no_conflict(self, rt):
        """不同时间的预约不冲突。"""
        cells = frozenset([Cell(5, 10)])
        rt.reserve_pose(t=3, cells=cells, robot_id="A_1")
        result = rt.reserve_pose(t=4, cells=cells, robot_id="B_1")
        assert result, "Different time reservations should not conflict"

    def test_reserve_different_cell_no_conflict(self, rt):
        """不同位置的同时间预约不冲突。"""
        rt.reserve_pose(t=3, cells=frozenset([Cell(5, 10)]), robot_id="A_1")
        result = rt.reserve_pose(t=3, cells=frozenset([Cell(6, 10)]), robot_id="B_1")
        assert result, "Different cell reservations should not conflict"


# ── is_pose_free ─────────────────────────────────────────────────────────


class TestPoseFree:
    """is_pose_free 查询测试。"""

    def test_free_when_empty(self, rt):
        """空表返回 True。"""
        assert rt.is_pose_free(t=3, cells=frozenset([Cell(5, 10)]))

    def test_not_free_when_reserved(self, rt):
        """有预约时返回 False。"""
        cells = frozenset([Cell(5, 10)])
        rt.reserve_pose(t=3, cells=cells, robot_id="A_1")
        assert not rt.is_pose_free(t=3, cells=cells)

    def test_free_for_own_robot(self, rt):
        """exclude_robot 参数排除自己后应返回 True。"""
        cells = frozenset([Cell(5, 10)])
        rt.reserve_pose(t=3, cells=cells, robot_id="A_1")
        assert rt.is_pose_free(t=3, cells=cells, exclude_robot="A_1")

    def test_not_free_for_other_robot(self, rt):
        """不排除时其他机器人的预约也算冲突。"""
        cells = frozenset([Cell(5, 10)])
        rt.reserve_pose(t=3, cells=cells, robot_id="A_1")
        assert not rt.is_pose_free(t=3, cells=cells, exclude_robot="B_1")


# ── 扫掠预约 ────────────────────────────────────────────────────────────


class TestSweepReservation:
    """reserve_swept / is_swept_free 测试。"""

    def test_sweep_reserve_success(self, rt, footprint):
        """扫掠预约成功。"""
        swept = footprint.cells_at(Cell(5, 10)) | footprint.cells_at(Cell(5, 11))
        result = rt.reserve_swept(t_start=5, t_end=6, cells=swept, robot_id="A_1")
        assert result, "Sweep reservation should succeed"

    def test_sweep_is_free_empty(self, rt):
        """空表时扫掠区域空闲。"""
        cells = frozenset([Cell(5, 10)])
        assert rt.is_swept_free(t_start=5, t_end=6, cells=cells)

    def test_sweep_not_free_when_reserved(self, rt, footprint):
        """有扫掠预约后不空闲。"""
        swept = footprint.cells_at(Cell(5, 10)) | footprint.cells_at(Cell(5, 11))
        rt.reserve_swept(t_start=5, t_end=6, cells=swept, robot_id="A_1")
        assert not rt.is_swept_free(t_start=5, t_end=6, cells=swept)

    def test_sweep_free_non_overlapping_interval(self, rt, footprint):
        """非重叠时间区间的扫掠不冲突。"""
        swept = footprint.cells_at(Cell(5, 10)) | footprint.cells_at(Cell(5, 11))
        rt.reserve_swept(t_start=5, t_end=6, cells=swept, robot_id="A_1")
        # 时间 [7,8) 不重叠
        assert rt.is_swept_free(t_start=7, t_end=8, cells=swept)

    def test_sweep_free_for_own_robot(self, rt, footprint):
        """自己的扫掠不算冲突。"""
        swept = footprint.cells_at(Cell(5, 10)) | footprint.cells_at(Cell(5, 11))
        rt.reserve_swept(t_start=5, t_end=6, cells=swept, robot_id="A_1")
        assert rt.is_swept_free(t_start=5, t_end=6, cells=swept, exclude_robot="A_1")

    def test_transition_cannot_sweep_through_stationary_robot(self, rt, footprint):
        """移动扫掠区必须与其他机器人的姿态预约交叉检查。"""
        blocker = footprint.cells_at(Cell(5, 10))
        rt.reserve_pose(t=5, cells=blocker, robot_id="B_1")
        rt.reserve_pose(t=6, cells=blocker, robot_id="B_1")
        swept = footprint.cells_at(Cell(4, 10)) | footprint.cells_at(Cell(5, 10))
        destination = footprint.cells_at(Cell(5, 10))
        assert not rt.is_transition_free(5, 6, swept, destination, "A_1")

    def test_same_start_time_keeps_independent_sweep_end_times(self, rt):
        """同一开始时刻的多条扫掠预约不能共享/覆盖结束时间。"""
        a = frozenset([Cell(1, 1)])
        b = frozenset([Cell(8, 8)])
        assert rt.reserve_swept(5, 6, a, "A_1")
        assert rt.reserve_swept(5, 10, b, "B_1")
        assert rt.is_swept_free(7, 8, a)
        assert not rt.is_swept_free(7, 8, b)


# ── 服务锁 ──────────────────────────────────────────────────────────────


class TestServiceLock:
    """reserve_service / is_service_free 测试。"""

    def test_service_reserve_success(self, rt):
        """服务锁预约成功。"""
        result = rt.reserve_service(
            machine_id="M_y3_x2", t_start=10, t_end=16, robot_id="A_1"
        )
        assert result, "Service lock should succeed"

    def test_service_is_free_empty(self, rt):
        """空表时服务空闲。"""
        assert rt.is_service_free("M_y3_x2", t_start=10, t_end=16)

    def test_service_not_free_when_locked(self, rt):
        """被锁后不空闲。"""
        rt.reserve_service("M_y3_x2", t_start=10, t_end=16, robot_id="A_1")
        assert not rt.is_service_free("M_y3_x2", t_start=10, t_end=16)

    def test_service_free_non_overlapping(self, rt):
        """非重叠时间区间的服务锁不冲突。"""
        rt.reserve_service("M_y3_x2", t_start=10, t_end=16, robot_id="A_1")
        assert rt.is_service_free("M_y3_x2", t_start=20, t_end=26)

    def test_service_lock_blocks_other_robot(self, rt):
        """服务锁阻止其他机器人。"""
        rt.reserve_service("M_y3_x2", t_start=10, t_end=16, robot_id="A_1")
        result = rt.reserve_service("M_y3_x2", t_start=12, t_end=18, robot_id="B_1")
        assert not result, "B_1 should not be able to lock same machine in overlapping interval"


# ── 冲突检测 ────────────────────────────────────────────────────────────


class TestConflictDetection:
    """get_conflicts_at / get_swept_conflicts 测试。"""

    def test_no_conflicts_when_empty(self, rt):
        """空表无冲突。"""
        conflicts = rt.get_conflicts_at(t=3, cells=frozenset([Cell(5, 10)]))
        assert conflicts == []

    def test_conflict_detected(self, rt):
        """冲突被检测到。"""
        cells = frozenset([Cell(5, 10)])
        rt.reserve_pose(t=3, cells=cells, robot_id="A_1")
        conflicts = rt.get_conflicts_at(t=3, cells=cells)
        assert "A_1" in conflicts

    def test_conflict_excludes_self(self, rt):
        """exclude_robot 排除自己。"""
        cells = frozenset([Cell(5, 10)])
        rt.reserve_pose(t=3, cells=cells, robot_id="A_1")
        conflicts = rt.get_conflicts_at(t=3, cells=cells, exclude_robot="A_1")
        assert conflicts == []

    def test_swept_conflicts_detected(self, rt, footprint):
        """扫掠冲突被检测到。"""
        swept = footprint.cells_at(Cell(5, 10)) | footprint.cells_at(Cell(5, 11))
        rt.reserve_swept(t_start=5, t_end=6, cells=swept, robot_id="A_1")
        conflicts = rt.get_swept_conflicts(t_start=5, t_end=6, cells=swept)
        assert "A_1" in conflicts


# ── release_future ──────────────────────────────────────────────────────


class TestReleaseFuture:
    """release_future 测试。"""

    def test_release_future_pose(self, rt):
        """释放后预约不再存在。"""
        cells = frozenset([Cell(5, 10)])
        rt.reserve_pose(t=5, cells=cells, robot_id="A_1")
        rt.reserve_pose(t=10, cells=cells, robot_id="A_1")
        rt.reserve_pose(t=15, cells=cells, robot_id="A_1")

        rt.release_future("A_1", from_time=10)

        # t=5 仍预约
        assert not rt.is_pose_free(t=5, cells=cells)
        # t=10, 15 已释放
        assert rt.is_pose_free(t=10, cells=cells)
        assert rt.is_pose_free(t=15, cells=cells)

    def test_release_future_only_own(self, rt):
        """只释放自己的。"""
        cells_a = frozenset([Cell(5, 10)])
        cells_b = frozenset([Cell(6, 10)])
        rt.reserve_pose(t=10, cells=cells_a, robot_id="A_1")
        rt.reserve_pose(t=10, cells=cells_b, robot_id="B_1")

        rt.release_future("A_1", from_time=10)
        assert rt.is_pose_free(t=10, cells=cells_a)
        assert not rt.is_pose_free(t=10, cells=cells_b)

    def test_release_future_swept(self, rt, footprint):
        """release_future 也释放扫掠预约。"""
        swept = footprint.cells_at(Cell(5, 10)) | footprint.cells_at(Cell(5, 11))
        rt.reserve_swept(t_start=10, t_end=11, cells=swept, robot_id="A_1")

        rt.release_future("A_1", from_time=10)
        assert rt.is_swept_free(t_start=10, t_end=11, cells=swept)


# ── clear / 统计 ────────────────────────────────────────────────────────


class TestClearAndStats:
    """clear, pose_count, swept_count 测试。"""

    def test_pose_count_initial(self, rt):
        assert rt.pose_count() == 0

    def test_pose_count_after_reservation(self, rt):
        rt.reserve_pose(t=3, cells=frozenset([Cell(5, 10)]), robot_id="A_1")
        assert rt.pose_count() == 1

    def test_swept_count(self, rt, footprint):
        swept = footprint.cells_at(Cell(5, 10)) | footprint.cells_at(Cell(5, 11))
        rt.reserve_swept(t_start=5, t_end=6, cells=swept, robot_id="A_1")
        assert rt.swept_count() == 1

    def test_clear_removes_all(self, rt, footprint):
        """clear 后所有统计归零。"""
        rt.reserve_pose(t=3, cells=frozenset([Cell(5, 10)]), robot_id="A_1")
        swept = footprint.cells_at(Cell(5, 10)) | footprint.cells_at(Cell(5, 11))
        rt.reserve_swept(t_start=5, t_end=6, cells=swept, robot_id="A_1")
        rt.reserve_service("M_y3_x2", t_start=10, t_end=16, robot_id="A_1")

        rt.clear()
        assert rt.pose_count() == 0
        assert rt.swept_count() == 0
