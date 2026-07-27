"""路径层向高层调度器返回的结构化冲突。"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PlanningConflict:
    robot_id: str
    conflicting_operation_ids: tuple[str, ...]
    conflicting_time_interval: tuple[int, int]
    minimum_required_delay: int
    suggested_precedence_constraint: tuple[str, str, int] | None
    conflict_type: str = "planning_failed"
    conflicting_robot_id: str | None = None
    source_event: str | None = None
    details: dict | None = None
