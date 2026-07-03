"""机器人 footprint 与碰撞检测单元测试 v4.0"""

from __future__ import annotations

import numpy as np
import pytest

from src.domain.enums import Action, TerrainCode
from src.domain.models import Cell, Footprint
from src.domain.validation import FootprintValidator


# ——— Fixtures ———————————————————————————————————————————————————————————


@pytest.fixture(scope="module")
def footprint() -> Footprint:
    return Footprint.default_2x4()


@pytest.fixture(scope="module")
def empty_terrain() -> np.ndarray:
    """29x25 全 INTERNAL_ROAD 地形（无障碍物）。"""
    return np.full((29, 25), TerrainCode.INTERNAL_ROAD.value, dtype=np.int32)


@pytest.fixture(scope="module")
def trunk_terrain() -> np.ndarray:
    """29x25: y<24 为 INTERNAL_ROAD，y>=24 为 TRUNK_ROAD。"""
    t = np.full((29, 25), TerrainCode.INTERNAL_ROAD.value, dtype=np.int32)
    t[23:, :] = TerrainCode.TRUNK_ROAD.value
    return t


@pytest.fixture(scope="module")
def obstacle_terrain() -> np.ndarray:
    """29x25: 第 10 列 y<24 为障碍物。"""
    t = np.full((29, 25), TerrainCode.INTERNAL_ROAD.value, dtype=np.int32)
    t[23:, :] = TerrainCode.TRUNK_ROAD.value
    t[:23, 9] = TerrainCode.OBSTACLE.value  # col 10 (1-indexed)
    return t


# ——— Footprint 结构 —————————————————————————————————————————————————————


class TestFootprintStructure:
    def test_cell_count(self, footprint: Footprint):
        """2x4 footprint 应有 8 个偏移。"""
        assert len(footprint.offsets) == 8

    def test_dimensions(self, footprint: Footprint):
        """检查 2x4 footprint 的范围。"""
        xs = {off.x for off in footprint.offsets}
        ys = {off.y for off in footprint.offsets}
        assert xs == {0, 1}, f"Expected x in {{0, 1}}, got {xs}"
        assert ys == {0, 1, 2, 3}, f"Expected y in {{0, 1, 2, 3}}, got {ys}"

    def test_work_zone(self):
        """work_zone 应为顶部两个格。"""
        wz = Footprint.work_zone()
        assert wz[0] == Cell(0, 0)
        assert wz[1] == Cell(1, 0)

    def test_cells_at(self, footprint: Footprint):
        """cells_at 应返回正确的 8 个占用格。"""
        anchor = Cell(5, 10)
        cells = footprint.cells_at(anchor)
        assert len(cells) == 8
        for off in footprint.offsets:
            assert (anchor + off) in cells


# ——— 姿态合法性 ——————————————————————————————————————————————————————


class TestValidPose:
    def test_valid_pose_center(self, footprint: Footprint, empty_terrain: np.ndarray):
        """地图中心的姿态应有效。"""
        assert FootprintValidator.is_valid_pose(Cell(5, 5), footprint, empty_terrain)

    def test_valid_pose_at_bounds(self, footprint: Footprint, empty_terrain: np.ndarray):
        """边界上的有效姿态。"""
        # 右下角边界：x=24（footprint x=24,25），y=26（footprint y=26..29）
        assert FootprintValidator.is_valid_pose(Cell(24, 26), footprint, empty_terrain)

    def test_valid_pose_top_left_corner(self, footprint: Footprint, empty_terrain: np.ndarray):
        """左上角 (1,1) 应有效。"""
        assert FootprintValidator.is_valid_pose(Cell(1, 1), footprint, empty_terrain)

    def test_invalid_pose_x_out_of_bounds(self, footprint: Footprint, empty_terrain: np.ndarray):
        """x 出界应无效: x=25 会使 footprint x 达到 26 > 25。"""
        assert not FootprintValidator.is_valid_pose(Cell(25, 1), footprint, empty_terrain)

    def test_invalid_pose_y_out_of_bounds(self, footprint: Footprint, empty_terrain: np.ndarray):
        """y 出界应无效: y=27 会使 footprint y 达到 30 > 29。"""
        assert not FootprintValidator.is_valid_pose(Cell(1, 27), footprint, empty_terrain)

    def test_invalid_pose_negative_x(self, footprint: Footprint, empty_terrain: np.ndarray):
        """x=0 出界。"""
        assert not FootprintValidator.is_valid_pose(Cell(0, 1), footprint, empty_terrain)

    def test_invalid_pose_negative_y(self, footprint: Footprint, empty_terrain: np.ndarray):
        """y=0 出界。"""
        assert not FootprintValidator.is_valid_pose(Cell(1, 0), footprint, empty_terrain)

    def test_invalid_pose_obstacle(self, footprint: Footprint, obstacle_terrain: np.ndarray):
        """触碰障碍物应无效。"""
        # anchor (10, 5): footprint covers x=10,11 — col=10 is obstacle
        assert not FootprintValidator.is_valid_pose(Cell(10, 5), footprint, obstacle_terrain)

    def test_valid_near_obstacle(self, footprint: Footprint, obstacle_terrain: np.ndarray):
        """不碰障碍物时应有效。"""
        # anchor (8, 5): footprint covers x=8,9 — col=10 obstacle not hit
        assert FootprintValidator.is_valid_pose(Cell(8, 5), footprint, obstacle_terrain)


# ——— 动作识别 ———————————————————————————————————————————————————————


class TestActionClassification:
    def test_up(self):
        assert FootprintValidator.classify_action(Cell(5, 5), Cell(5, 4)) == Action.UP

    def test_down(self):
        assert FootprintValidator.classify_action(Cell(5, 5), Cell(5, 6)) == Action.DOWN

    def test_left(self):
        assert FootprintValidator.classify_action(Cell(5, 5), Cell(4, 5)) == Action.LEFT

    def test_right(self):
        assert FootprintValidator.classify_action(Cell(5, 5), Cell(6, 5)) == Action.RIGHT

    def test_wait(self):
        assert FootprintValidator.classify_action(Cell(5, 5), Cell(5, 5)) == Action.WAIT

    def test_diagonal_invalid(self):
        assert FootprintValidator.classify_action(Cell(5, 5), Cell(6, 6)) is None

    def test_long_jump_invalid(self):
        assert FootprintValidator.classify_action(Cell(5, 5), Cell(5, 3)) is None


# ——— 转移合法性 ————————————————————————————————————————————————————


class TestTransition:
    def test_vertical_in_internal_allowed(
        self, footprint: Footprint, trunk_terrain: np.ndarray
    ):
        """内部区域 (y<24) 的上下移动应允许。"""
        valid, reason = FootprintValidator.is_valid_transition(
            Cell(5, 5), Cell(5, 6), footprint, trunk_terrain
        )
        assert valid, reason

    def test_horizontal_in_internal_rejected(
        self, footprint: Footprint, trunk_terrain: np.ndarray
    ):
        """内部区域水平移动应被拒绝。"""
        valid, reason = FootprintValidator.is_valid_transition(
            Cell(5, 5), Cell(6, 5), footprint, trunk_terrain
        )
        assert not valid
        assert "水平移动" in reason or "仅允许在主干道" in reason

    def test_horizontal_in_trunk_allowed(
        self, footprint: Footprint, trunk_terrain: np.ndarray
    ):
        """主干道 (y>=24) 水平移动应允许。"""
        valid, reason = FootprintValidator.is_valid_transition(
            Cell(5, 24), Cell(6, 24), footprint, trunk_terrain
        )
        assert valid, reason

    def test_horizontal_across_boundary_rejected(
        self, footprint: Footprint, trunk_terrain: np.ndarray
    ):
        """从内部到主干道的水平移动仍被拒绝（from_y=23<24）。"""
        valid, reason = FootprintValidator.is_valid_transition(
            Cell(5, 23), Cell(6, 23), footprint, trunk_terrain
        )
        assert not valid

    def test_wait_always_allowed(
        self, footprint: Footprint, trunk_terrain: np.ndarray
    ):
        """WAIT 应始终允许（只要起点有效）。"""
        valid, reason = FootprintValidator.is_valid_transition(
            Cell(5, 5), Cell(5, 5), footprint, trunk_terrain
        )
        assert valid, reason

    def test_transition_to_obstacle_rejected(
        self, footprint: Footprint, obstacle_terrain: np.ndarray
    ):
        """转移到障碍物上应被拒绝。"""
        # anchor (9, 5) -> (10, 5) 会碰 col=10 obstacle
        valid, reason = FootprintValidator.is_valid_transition(
            Cell(9, 5), Cell(10, 5), footprint, obstacle_terrain
        )
        assert not valid

    def test_transition_out_of_bounds_rejected(
        self, footprint: Footprint, trunk_terrain: np.ndarray
    ):
        """出界转移应被拒绝。"""
        valid, reason = FootprintValidator.is_valid_transition(
            Cell(24, 26), Cell(25, 26), footprint, trunk_terrain
        )
        assert not valid

    def test_vertical_in_trunk_allowed(
        self, footprint: Footprint, trunk_terrain: np.ndarray
    ):
        """主干道垂直移动应允许。"""
        valid, reason = FootprintValidator.is_valid_transition(
            Cell(5, 25), Cell(5, 26), footprint, trunk_terrain
        )
        assert valid, reason


# ——— 扫掠区域 ——————————————————————————————————————————————————————


class TestSweptCells:
    def test_swept_union(self, footprint: Footprint):
        """扫掠区域 = 起点 ∪ 终点 footprint。"""
        from_cell = Cell(5, 5)
        to_cell = Cell(5, 6)
        swept = FootprintValidator.swept_cells(from_cell, to_cell, footprint)

        expected = footprint.cells_at(from_cell) | footprint.cells_at(to_cell)
        assert swept == expected

    def test_swept_wait_equals_start(self, footprint: Footprint):
        """WAIT 的扫掠区域 = 起点 footprint。"""
        from_cell = Cell(5, 5)
        swept = FootprintValidator.swept_cells(from_cell, from_cell, footprint)
        assert swept == footprint.cells_at(from_cell)

    def test_swept_count(self, footprint: Footprint):
        """上下移动扫掠区域最多 16 格（有重叠时更少）。"""
        from_cell = Cell(5, 5)
        to_cell = Cell(5, 6)  # DOWN
        swept = FootprintValidator.swept_cells(from_cell, to_cell, footprint)

        # 上移 1 格：12 个格重叠（bottom 3 rows of from = top 3 rows of to）
        # 总共 8 + 8 - 6 = 10 个不同格
        assert len(swept) == 10, f"Expected 10 swept cells, got {len(swept)}"


# ——— 工作区对齐 ————————————————————————————————————————————————————


class TestWorkZoneAlignment:
    def test_work_zone_covers_machine(self, footprint: Footprint):
        """work_zone (top 2 cells) 应对齐离心机 2 格。"""
        # 离心机在 (5,3) 和 (6,3)
        # 机器人锚点 = 离心机左侧格 (5,3)
        # work_zone.top = 机器人锚点 row = 3
        # work_zone cells: (5,3) 和 (6,3) ✓
        anchor = Cell(5, 3)
        wz = Footprint.work_zone()
        wz_cells = {anchor + wz[0], anchor + wz[1]}
        machine_cells = {Cell(5, 3), Cell(6, 3)}
        assert wz_cells == machine_cells

    def test_work_zone_above_body(self, footprint: Footprint):
        """work_zone 应在机器人本体最上方两格。"""
        anchor = Cell(5, 10)
        wz = Footprint.work_zone()
        wz_cells = {anchor + wz[0], anchor + wz[1]}
        body_cells = footprint.cells_at(anchor)

        # work_zone cells 应是本体 cells 的子集（前两格的 row=10）
        assert wz_cells.issubset(body_cells)
        # work_zone cells 的 y 坐标应为锚点行（最小 y）
        for c in wz_cells:
            assert c.y == anchor.y
