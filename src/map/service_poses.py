"""服务姿态计算器 v4.0

计算机器人服务每台离心机时的有效锚点位置。
"""

from __future__ import annotations

from ..domain.models import Cell, Footprint, Machine


class ServicePoseCalculator:
    """计算机器人服务离心机时的有效锚点位置。

    机器人锚点需要使工作区（work_zone: top 2 cells）对齐离心机（2 格横向）。
    离心机在 (y, x) 和 (y, x+1)，工作区需要覆盖这两个格。
    """

    @staticmethod
    def compute_service_anchor(machine: Machine, footprint: Footprint) -> Cell:
        """默认服务锚点 = 离心机左侧格坐标。

        验证：footprint 的 work_zone（top 2 cells）能覆盖离心机的 2 个格。
        机器人锚点 = 离心机左侧格，则：
          - work_zone 格 0: anchor + (0, 0) = 离心机左侧格 ✓
          - work_zone 格 1: anchor + (1, 0) = 离心机右侧格 ✓

        注意：机器人实际放置在离心机下方（即 y+1 处），此处返回的
        service_anchor 是"上方对齐点"，用于定义操作发生的位置。

        Args:
            machine: 离心机对象
            footprint: 机器人 footprint

        Returns:
            服务锚点 Cell

        Raises:
            ValueError: 如果 work_zone 无法覆盖离心机
        """
        left_cell = machine.cells[0]  # 离心机左侧格
        wz = Footprint.work_zone()  # (Cell(0,0), Cell(1,0))

        # 验证 work_zone 覆盖离心机
        wz_cell0 = left_cell + wz[0]  # should equal machine.cells[0]
        wz_cell1 = left_cell + wz[1]  # should equal machine.cells[1]

        if wz_cell0 != machine.cells[0] or wz_cell1 != machine.cells[1]:
            raise ValueError(
                f"Work zone alignment failed for machine {machine.machine_id}: "
                f"work_zone covers ({wz_cell0.x},{wz_cell0.y}) and "
                f"({wz_cell1.x},{wz_cell1.y}), but machine cells are "
                f"({machine.cells[0].x},{machine.cells[0].y}) and "
                f"({machine.cells[1].x},{machine.cells[1].y})"
            )

        return left_cell

    @staticmethod
    def compute_all_service_anchors(
        machines: dict[str, Machine], footprint: Footprint
    ) -> dict[str, Cell]:
        """为所有离心机计算服务锚点。

        Args:
            machines: 离心机字典，key=machine_id
            footprint: 机器人 footprint

        Returns:
            {machine_id: service_anchor} 字典
        """
        return {
            mid: ServicePoseCalculator.compute_service_anchor(m, footprint)
            for mid, m in machines.items()
        }
