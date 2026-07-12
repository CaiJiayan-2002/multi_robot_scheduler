"""使用 footprint-aware 静态 A* 构建 CP-SAT 旅行时间。"""
from __future__ import annotations

from ..domain.models import Cell, Footprint, Operation, RobotSpec
from ..map.pose_graph import PoseGraph
from ..planning.static_astar import StaticAStar


def build_operation_travel_times(
    pose_graph: PoseGraph,
    footprint: Footprint,
    operations: dict[str, Operation],
    robots: dict[str, RobotSpec],
) -> dict[tuple[str, str, str], int]:
    """返回 (robot_id, from_operation|START, to_operation|END) -> 距离。"""
    astar = StaticAStar(pose_graph)
    anchors = {op_id: op.service_anchor for op_id, op in operations.items()}
    result: dict[tuple[str, str, str], int] = {}

    def distance(start: Cell, end: Cell) -> int:
        planned = astar.plan(start, end)
        if planned is None:
            raise ValueError(f"unreachable service poses: {start} -> {end}")
        return planned[1]

    # 机器相同的三个操作共享服务点，按 anchor 对缓存，避免重复 A*。
    cache: dict[tuple[Cell, Cell], int] = {}
    def cached(start: Cell, end: Cell) -> int:
        key = (start, end)
        if key not in cache:
            cache[key] = distance(start, end)
        return cache[key]

    for rid, robot in robots.items():
        for op_id, anchor in anchors.items():
            result[(rid, "START", op_id)] = cached(robot.start_anchor, anchor)
            result[(rid, op_id, "END")] = cached(anchor, robot.start_anchor)
        for from_id, from_anchor in anchors.items():
            for to_id, to_anchor in anchors.items():
                if from_id != to_id:
                    result[(rid, from_id, to_id)] = cached(from_anchor, to_anchor)
    return result

