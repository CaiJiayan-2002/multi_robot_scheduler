"""固定地图生成器 v4.0

生成符合 v4.0 设计文档的固定 25x29 地图。
"""

from __future__ import annotations

import numpy as np

from ..domain.enums import MachineState, OperationType, RobotType, TerrainCode
from ..domain.models import Cell, Machine, Operation


class FixedMap:
    """生成 v4.0 定义的固定 25x29 地图。

    三层结构：
    - terrain: 2D numpy array (0=obstacle, 1=internal_road, 2=trunk_road)
    - machines: dict[str, Machine] (48台离心机)
    - operations: dict[str, Operation] (144个操作)
    """

    # — 地图几何常量 ——————————————————————————————
    WIDTH: int = 25
    HEIGHT: int = 29
    INTERNAL_ROWS = (1, 23)  # 1-indexed, inclusive
    TRUNK_ROWS = (24, 29)
    OBSTACLE_COLUMNS = (4, 10, 16, 22)  # 1-indexed
    TRUNK_Y_THRESHOLD: int = 24

    # — 离心机布局常量 ————————————————————————————
    MACHINE_ROWS = (3, 7, 11, 15, 19, 23)
    MACHINE_X_STARTS = (2, 5, 8, 11, 14, 17, 20, 23)

    # — 操作类型与时长 ————————————————————————————
    _OP_SPECS = {
        OperationType.DISASSEMBLE: {"robot_type": RobotType.A, "duration": 6},
        OperationType.INSPECT: {"robot_type": RobotType.B, "duration": 10},
        OperationType.INSTALL: {"robot_type": RobotType.A, "duration": 6},
    }

    _OP_CHAIN = (
        OperationType.DISASSEMBLE,
        OperationType.INSPECT,
        OperationType.INSTALL,
    )

    def __init__(self, config_path: str | None = None) -> None:
        """初始化固定地图。

        Args:
            config_path: 配置文件路径（保留参数，v4.0 使用硬编码常量）
        """
        self._config_path = config_path
        self._terrain: np.ndarray | None = None
        self._machines: dict[str, Machine] | None = None
        self._operations: dict[str, Operation] | None = None

    # ------------------------------------------------------------------
    def build(self) -> tuple[np.ndarray, dict[str, Machine], dict[str, Operation]]:
        """生成固定地图。

        Returns:
            terrain: shape=(29, 25)，0-indexed，值域 0/1/2
            machines: key=machine_id，48台离心机
            operations: key=operation_id，144个操作
        """
        self._terrain = self._generate_terrain()
        self._machines = self.generate_machines()
        self._operations = self.generate_operations(self._machines)
        return self._terrain, self._machines, self._operations

    @property
    def terrain(self) -> np.ndarray | None:
        return self._terrain

    @property
    def machines(self) -> dict[str, Machine] | None:
        return self._machines

    @property
    def operations(self) -> dict[str, Operation] | None:
        return self._operations

    # ——— 地形生成 —————————————————————————————————————————————————————
    def _generate_terrain(self) -> np.ndarray:
        """生成 29x25 地形矩阵。

        规则：
        - 主干道 (y >= 24): 全部为 TRUNK_ROAD (2)
        - 内部区域 (y < 24): 障碍列 (4,10,16,22) 为 OBSTACLE (0)，其余为 INTERNAL_ROAD (1)
        """
        terrain = np.full((self.HEIGHT, self.WIDTH), TerrainCode.INTERNAL_ROAD.value, dtype=np.int32)

        # 主干道
        trunk_start_0 = self.TRUNK_Y_THRESHOLD - 1  # 0-indexed
        terrain[trunk_start_0:, :] = TerrainCode.TRUNK_ROAD.value

        # 内部障碍列
        for col_1 in self.OBSTACLE_COLUMNS:
            col_0 = col_1 - 1  # 0-indexed
            terrain[:trunk_start_0, col_0] = TerrainCode.OBSTACLE.value

        return terrain

    # ——— 离心机生成 —————————————————————————————————————————————————
    @staticmethod
    def generate_machines() -> dict[str, Machine]:
        """在 rows=[3,7,11,15,19,23], x_starts=[2,5,8,11,14,17,20,23]
        生成 48 台离心机。每台占 (x,y) 和 (x+1,y)。

        ID 格式: M_y{row}_x{x}
        """
        machines: dict[str, Machine] = {}
        for row in FixedMap.MACHINE_ROWS:
            for x_start in FixedMap.MACHINE_X_STARTS:
                machine_id = f"M_y{row}_x{x_start}"
                cells = (Cell(x_start, row), Cell(x_start + 1, row))
                machines[machine_id] = Machine(
                    machine_id=machine_id,
                    cells=cells,
                    row=row,
                    state=MachineState.PENDING_DISASSEMBLY,
                )
        return machines

    # ——— 操作生成 ———————————————————————————————————————————————————
    @staticmethod
    def generate_operations(machines: dict[str, Machine]) -> dict[str, Operation]:
        """为每台离心机生成 3 个操作。

        操作链: DISASSEMBLE -> INSPECT -> INSTALL
        - DISASSEMBLE: A 机器人，6 时间单位
        - INSPECT:     B 机器人，10 时间单位
        - INSTALL:     A 机器人，6 时间单位

        操作 ID 格式: {machine_id}_D / {machine_id}_I / {machine_id}_R
        service_anchor = 离心机左侧格

        Args:
            machines: 离心机字典

        Returns:
            144 个 Operation 的字典
        """
        operations: dict[str, Operation] = {}

        # 操作类型到短后缀的映射
        suffix_map = {
            OperationType.DISASSEMBLE: "D",
            OperationType.INSPECT: "I",
            OperationType.INSTALL: "R",
        }

        for machine_id, machine in machines.items():
            left_cell = machine.cells[0]  # 左侧格作为 service_anchor

            prev_op_id: str | None = None
            for op_type in FixedMap._OP_CHAIN:
                spec = FixedMap._OP_SPECS[op_type]
                suffix = suffix_map[op_type]
                op_id = f"{machine_id}_{suffix}"

                operations[op_id] = Operation(
                    operation_id=op_id,
                    machine_id=machine_id,
                    operation_type=op_type,
                    eligible_robot_type=spec["robot_type"],
                    duration=spec["duration"],
                    service_anchor=left_cell,
                    predecessor_id=prev_op_id,
                )
                prev_op_id = op_id

        return operations
