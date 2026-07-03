"""有效姿态图 v4.0

预计算所有有效姿态及它们之间的转移代价。

节点 = 有效的锚点 Cell（footprint 全在界内且不碰障碍物）
边 = 5 种动作的合法转移，代价 = 1
"""

from __future__ import annotations

import numpy as np

from ..domain.enums import Action
from ..domain.models import Cell, Footprint
from ..domain.validation import FootprintValidator


class PoseGraph:
    """预计算所有有效姿态及它们之间的转移代价。

    Attributes:
        terrain: 地形矩阵
        footprint: 机器人 footprint
        trunk_y_threshold: 主干道起始 y 坐标
        valid_poses: 所有有效锚点集合
        adjacency: 邻接表 {anchor: [(neighbor, cost), ...]}
        validator: FootprintValidator 实例（供查询使用）
    """

    def __init__(
        self,
        terrain: np.ndarray,
        footprint: Footprint,
        trunk_y_threshold: int = 24,
    ) -> None:
        """初始化姿态图（不立即构建）。

        Args:
            terrain: 2D numpy array，shape=(H, W)
            footprint: 机器人 footprint
            trunk_y_threshold: 主干道起始 y 坐标（1-indexed）
        """
        self.terrain = terrain
        self.footprint = footprint
        self.trunk_y_threshold = trunk_y_threshold
        self.H, self.W = terrain.shape

        self.valid_poses: set[Cell] = set()
        self.adjacency: dict[Cell, list[tuple[Cell, int]]] = {}

    # ------------------------------------------------------------------
    def build(self) -> None:
        """枚举所有有效姿态，建立邻接表。

        时间复杂度: O(H * W * 5 * footprint_size)
        对于 29x25 地图约 30k 次检查，可瞬时完成。
        """
        self.valid_poses.clear()
        self.adjacency.clear()

        # 1. 枚举所有有效锚点
        #    机器人宽 2 高 4，所以锚点范围:
        #    x in [1, W-1], y in [1, H-3]
        max_x = self.W - 1  # footprint 宽 2，锚点 x 最大 = W-1
        max_y = self.H - 3  # footprint 高 4，锚点 y 最大 = H-3

        for y_1 in range(1, max_y + 1):  # 1-indexed
            for x_1 in range(1, max_x + 1):
                anchor = Cell(x_1, y_1)
                if FootprintValidator.is_valid_pose(anchor, self.footprint, self.terrain):
                    self.valid_poses.add(anchor)

        # 2. 为每个有效姿态构建邻接边
        for anchor in self.valid_poses:
            neighbors: list[tuple[Cell, int]] = []

            # WAIT 总是合法
            neighbors.append((anchor, 1))

            for action in (Action.UP, Action.DOWN, Action.LEFT, Action.RIGHT):
                dx, dy = action.value
                neighbor = Cell(anchor.x + dx, anchor.y + dy)

                valid, _ = FootprintValidator.is_valid_transition(
                    anchor, neighbor, self.footprint, self.terrain,
                    trunk_y_threshold=self.trunk_y_threshold,
                )
                if valid:
                    neighbors.append((neighbor, 1))

            self.adjacency[anchor] = neighbors

    # ------------------------------------------------------------------
    def get_neighbors(self, anchor: Cell) -> list[tuple[Cell, int]]:
        """返回从 anchor 出发的合法邻居及代价。

        Args:
            anchor: 锚点

        Returns:
            [(neighbor_anchor, cost), ...] 列表
        """
        return self.adjacency.get(anchor, [])

    # ------------------------------------------------------------------
    def is_valid_pose(self, anchor: Cell) -> bool:
        """检查锚点是否为有效姿态。

        Args:
            anchor: 锚点

        Returns:
            True 如果 anchor 在有效姿态集中
        """
        return anchor in self.valid_poses

    # ------------------------------------------------------------------
    def node_count(self) -> int:
        """返回有效姿态数量。"""
        return len(self.valid_poses)

    # ------------------------------------------------------------------
    def edge_count(self) -> int:
        """返回总边数。"""
        return sum(len(v) for v in self.adjacency.values())
