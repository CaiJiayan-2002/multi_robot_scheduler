"""固定地图单元测试 v4.0"""

from __future__ import annotations

import numpy as np
import pytest

from src.domain.enums import MachineState, OperationType, RobotType, TerrainCode
from src.domain.models import Footprint
from src.map.fixed_map import FixedMap


# ——— Fixtures ———————————————————————————————————————————————————————————


@pytest.fixture(scope="module")
def fixed_map() -> FixedMap:
    """构建一次，所有测试共享。"""
    fm = FixedMap()
    fm.build()
    return fm


@pytest.fixture(scope="module")
def terrain(fixed_map: FixedMap) -> np.ndarray:
    assert fixed_map.terrain is not None
    return fixed_map.terrain


@pytest.fixture(scope="module")
def machines(fixed_map: FixedMap) -> dict:
    assert fixed_map.machines is not None
    return fixed_map.machines


@pytest.fixture(scope="module")
def operations(fixed_map: FixedMap) -> dict:
    assert fixed_map.operations is not None
    return fixed_map.operations


# ——— 地图尺寸 —————————————————————————————————————————————————————————


class TestMapDimensions:
    def test_terrain_shape(self, terrain: np.ndarray):
        """地形矩阵应为 29 行 x 25 列。"""
        assert terrain.shape == (29, 25), f"Expected (29, 25), got {terrain.shape}"

    def test_width_constant(self):
        """WIDTH 常量 = 25。"""
        assert FixedMap.WIDTH == 25

    def test_height_constant(self):
        """HEIGHT 常量 = 29。"""
        assert FixedMap.HEIGHT == 29


# ——— 障碍列 —————————————————————————————————————————————————————————


class TestObstacleColumns:
    OBSTACLE_COLS = (4, 10, 16, 22)

    def test_obstacle_in_internal_rows(self, terrain: np.ndarray):
        """内部区域 (y<24) 的障碍列应为 OBSTACLE(0)。"""
        trunk_start_0 = 23  # 0-indexed: y=24 → row 23
        for col_1 in self.OBSTACLE_COLS:
            col_0 = col_1 - 1
            for row_0 in range(trunk_start_0):
                assert terrain[row_0, col_0] == TerrainCode.OBSTACLE.value, (
                    f"Expected OBSTACLE at row={row_0}, col={col_0} "
                    f"but got {terrain[row_0, col_0]}"
                )

    def test_non_obstacle_in_internal_rows(self, terrain: np.ndarray):
        """内部区域非障碍列应为 INTERNAL_ROAD(1)。"""
        trunk_start_0 = 23
        non_obstacle = [c for c in range(1, 26) if c not in self.OBSTACLE_COLS]
        for col_1 in non_obstacle:
            col_0 = col_1 - 1
            for row_0 in range(trunk_start_0):
                assert terrain[row_0, col_0] == TerrainCode.INTERNAL_ROAD.value, (
                    f"Expected INTERNAL_ROAD at row={row_0}, col={col_0}"
                )

    def test_obstacle_not_in_trunk(self, terrain: np.ndarray):
        """主干道 (y>=24) 无障碍物。"""
        trunk_start_0 = 23
        for col_1 in self.OBSTACLE_COLS:
            col_0 = col_1 - 1
            for row_0 in range(trunk_start_0, 29):
                assert terrain[row_0, col_0] != TerrainCode.OBSTACLE.value, (
                    f"Unexpected OBSTACLE in trunk at row={row_0}, col={col_0}"
                )


# ——— 主干道 —————————————————————————————————————————————————————————


class TestTrunkRoad:
    def test_trunk_all_road(self, terrain: np.ndarray):
        """主干道 (y>=24) 所有格为 TRUNK_ROAD(2)。"""
        trunk_start_0 = 23  # y=24 -> 0-indexed row 23
        for row_0 in range(trunk_start_0, 29):
            for col_0 in range(25):
                assert terrain[row_0, col_0] == TerrainCode.TRUNK_ROAD.value, (
                    f"Expected TRUNK_ROAD at row={row_0}, col={col_0}"
                )

    def test_internal_not_trunk(self, terrain: np.ndarray):
        """内部区域 (y<24) 不是 TRUNK_ROAD。"""
        for row_0 in range(23):
            for col_0 in range(25):
                assert terrain[row_0, col_0] != TerrainCode.TRUNK_ROAD.value


# ——— 离心机 —————————————————————————————————————————————————————————


class TestMachines:
    def test_machine_count(self, machines: dict):
        """应为 48 台离心机。"""
        assert len(machines) == 48, f"Expected 48 machines, got {len(machines)}"

    def test_machine_cells_count(self, machines: dict):
        """每台离心机恰好 2 个格。"""
        for mid, m in machines.items():
            assert len(m.cells) == 2, f"Machine {mid} has {len(m.cells)} cells"

    def test_machine_cells_adjacent(self, machines: dict):
        """离心机的两个格应横向相邻: 同行，x 差 1。"""
        for mid, m in machines.items():
            c0, c1 = m.cells
            assert c0.y == c1.y, f"Machine {mid} cells not same row: {c0}, {c1}"
            assert c1.x - c0.x == 1, f"Machine {mid} cells not adjacent: {c0}, {c1}"

    def test_machine_rows(self, machines: dict):
        """所有离心机行号在指定集合中。"""
        expected_rows = set(FixedMap.MACHINE_ROWS)
        for mid, m in machines.items():
            assert m.row in expected_rows, f"Machine {mid} row {m.row} not in {expected_rows}"

    def test_machine_x_starts(self, machines: dict):
        """所有离心机起始 x 在指定集合中。"""
        expected_x = set(FixedMap.MACHINE_X_STARTS)
        for mid, m in machines.items():
            assert m.cells[0].x in expected_x, f"Machine {mid} x_start {m.cells[0].x} not in {expected_x}"

    def test_machine_positions_exact(self, machines: dict):
        """验证机器位置覆盖所有行列组合。"""
        expected_ids = set()
        for row in FixedMap.MACHINE_ROWS:
            for x_start in FixedMap.MACHINE_X_STARTS:
                expected_ids.add(f"M_y{row}_x{x_start}")
        actual_ids = set(machines.keys())
        assert actual_ids == expected_ids

    def test_machine_initial_state(self, machines: dict):
        """初始状态应为 PENDING_DISASSEMBLY。"""
        for mid, m in machines.items():
            assert m.state == MachineState.PENDING_DISASSEMBLY

    def test_each_row_has_8_machines(self, machines: dict):
        """每行应有恰好 8 台离心机。"""
        for row in FixedMap.MACHINE_ROWS:
            count = sum(1 for m in machines.values() if m.row == row)
            assert count == 8, f"Row {row} has {count} machines, expected 8"


# ——— 操作 —————————————————————————————————————————————————————————


class TestOperations:
    def test_operation_count(self, operations: dict):
        """应为 144 个操作 (48 x 3)。"""
        assert len(operations) == 144, f"Expected 144 operations, got {len(operations)}"

    def test_operation_types(self, operations: dict):
        """DISASSEMBLE/INSTALL 配 A，INSPECT 配 B。"""
        for op_id, op in operations.items():
            if op_id.endswith("_D"):  # DISASSEMBLE
                assert op.operation_type == OperationType.DISASSEMBLE
                assert op.eligible_robot_type == RobotType.A
                assert op.duration == 6
            elif op_id.endswith("_I"):  # INSPECT
                assert op.operation_type == OperationType.INSPECT
                assert op.eligible_robot_type == RobotType.B
                assert op.duration == 10
            elif op_id.endswith("_R"):  # INSTALL
                assert op.operation_type == OperationType.INSTALL
                assert op.eligible_robot_type == RobotType.A
                assert op.duration == 6
            else:
                pytest.fail(f"Unknown operation suffix: {op_id}")

    def test_operation_chain(self, operations: dict, machines: dict):
        """操作链 D -> I -> R 的前驱关系正确。"""
        for mid in machines:
            d_id = f"{mid}_D"
            i_id = f"{mid}_I"
            r_id = f"{mid}_R"

            assert operations[d_id].predecessor_id is None, f"{d_id} should have no predecessor"
            assert operations[i_id].predecessor_id == d_id
            assert operations[r_id].predecessor_id == i_id

    def test_service_anchor_is_left_cell(self, operations: dict, machines: dict):
        """service_anchor 应为离心机左侧格。"""
        for mid, m in machines.items():
            left_cell = m.cells[0]
            for suffix in ("_D", "_I", "_R"):
                op = operations[f"{mid}{suffix}"]
                assert op.service_anchor == left_cell

    def test_three_ops_per_machine(self, operations: dict, machines: dict):
        """每台离心机恰好 3 个操作。"""
        for mid in machines:
            ops_for_machine = [op for op_id, op in operations.items() if op.machine_id == mid]
            assert len(ops_for_machine) == 3, f"Machine {mid} has {len(ops_for_machine)} ops"
