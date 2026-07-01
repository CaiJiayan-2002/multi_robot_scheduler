"""Footprint 与过渡验证器 v4.0

验证机器人 2x4 footprint 的姿态合法性，以及姿态间的转移合法性。
"""

from __future__ import annotations

import numpy as np

from .enums import Action, TerrainCode
from .models import Cell, Footprint


class FootprintValidator:
    """验证 2x4 footprint 姿态是否合法。

    所有方法均为静态方法，无状态。
    """

    @staticmethod
    def is_valid_pose(
        anchor: Cell,
        footprint: Footprint,
        terrain: np.ndarray,
        terrain_codes: dict[int, int] | None = None,
    ) -> bool:
        """检查锚点处的 footprint 是否全部在界内且不碰障碍物。

        Args:
            anchor: 机器人锚点（左上角），1-indexed
            footprint: 机器人 footprint 偏移
            terrain: 2D numpy array，shape=(H, W)，0-indexed，
                     值 0=obstacle, 1=internal_road, 2=trunk_road
            terrain_codes: 地形编码映射（保留参数，v4.0 中障碍物固定为 0）

        Returns:
            True 如果所有 footprint 格都在界内且不碰障碍物
        """
        H, W = terrain.shape
        for offset in footprint.offsets:
            cell = anchor + offset
            # 检查边界（Cell 使用 1-indexed，terrain 使用 0-indexed）
            if not (1 <= cell.x <= W and 1 <= cell.y <= H):
                return False
            # 检查障碍物
            if terrain[cell.y - 1, cell.x - 1] == TerrainCode.OBSTACLE.value:
                return False
        return True

    @staticmethod
    def is_valid_transition(
        from_anchor: Cell,
        to_anchor: Cell,
        footprint: Footprint,
        terrain: np.ndarray,
        terrain_codes: dict[int, int] | None = None,
        trunk_y_threshold: int = 24,
    ) -> tuple[bool, str]:
        """检查从 from_anchor 到 to_anchor 的转移是否合法。

        规则：
        - 位移必须是 5 种动作之一 (UP/DOWN/LEFT/RIGHT/WAIT)
        - 水平移动必须在主干道 (y >= trunk_y_threshold)
        - 目标姿态必须有效
        - 扫掠区域（起点 footprint ∪ 终点 footprint）不能碰障碍物

        Args:
            from_anchor: 起始锚点
            to_anchor: 目标锚点
            footprint: 机器人 footprint
            terrain: 2D numpy array
            terrain_codes: 地形编码映射（保留参数）
            trunk_y_threshold: 主干道起始 y 坐标（1-indexed），默认 24

        Returns:
            (is_valid, reason) 元组
        """
        dx = to_anchor.x - from_anchor.x
        dy = to_anchor.y - from_anchor.y

        # —— 识别动作 ——————————————————————————————
        action: Action | None = None
        if dx == 0 and dy == -1:
            action = Action.UP
        elif dx == 0 and dy == 1:
            action = Action.DOWN
        elif dx == -1 and dy == 0:
            action = Action.LEFT
        elif dx == 1 and dy == 0:
            action = Action.RIGHT
        elif dx == 0 and dy == 0:
            action = Action.WAIT
        else:
            return False, f"非法位移: delta=({dx},{dy})，必须是单步动作或等待"

        # —— 水平移动限制 ——————————————————————————
        if action in (Action.LEFT, Action.RIGHT):
            if not (
                from_anchor.y >= trunk_y_threshold
                and to_anchor.y >= trunk_y_threshold
            ):
                return (
                    False,
                    f"水平移动 ({action.name}) 仅允许在主干道 y>={trunk_y_threshold}，"
                    f"当前 from_y={from_anchor.y}, to_y={to_anchor.y}",
                )

        # —— 目标姿态合法性 ————————————————————————
        if not FootprintValidator.is_valid_pose(to_anchor, footprint, terrain):
            return False, f"目标姿态无效: anchor=({to_anchor.x},{to_anchor.y})"

        # —— 扫掠区域检查 ——————————————————————————
        if action != Action.WAIT:
            swept = FootprintValidator.swept_cells(from_anchor, to_anchor, footprint)
            H, W = terrain.shape
            for cell in swept:
                if not (1 <= cell.x <= W and 1 <= cell.y <= H):
                    return False, f"扫掠区域出界: ({cell.x},{cell.y})"
                if terrain[cell.y - 1, cell.x - 1] == TerrainCode.OBSTACLE.value:
                    return False, f"扫掠区域有障碍物: ({cell.x},{cell.y})"

        return True, "OK"

    @staticmethod
    def swept_cells(
        from_anchor: Cell, to_anchor: Cell, footprint: Footprint
    ) -> frozenset[Cell]:
        """计算移动的扫掠区域 = 起点 footprint ∪ 终点 footprint。

        Args:
            from_anchor: 起始锚点
            to_anchor: 目标锚点
            footprint: 机器人 footprint

        Returns:
            起点和终点所有占用格的并集
        """
        return footprint.cells_at(from_anchor) | footprint.cells_at(to_anchor)

    @staticmethod
    def classify_action(from_anchor: Cell, to_anchor: Cell) -> Action | None:
        """识别两个锚点之间的动作类型。

        Args:
            from_anchor: 起始锚点
            to_anchor: 目标锚点

        Returns:
            对应的 Action 枚举，或 None（非法位移）
        """
        dx = to_anchor.x - from_anchor.x
        dy = to_anchor.y - from_anchor.y

        if dx == 0 and dy == -1:
            return Action.UP
        elif dx == 0 and dy == 1:
            return Action.DOWN
        elif dx == -1 and dy == 0:
            return Action.LEFT
        elif dx == 1 and dy == 0:
            return Action.RIGHT
        elif dx == 0 and dy == 0:
            return Action.WAIT
        return None
