"""求解器正式模式配置。"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SolverConfig:
    solver_mode: str = "assignment_schedule"
    allow_fallback: bool = False
    max_time_seconds: int = 60
    makespan_tolerance: int = 0
    require_same_a_robot_for_disassemble_and_install: bool = False
    preferred_first_column: int | None = 2
    preferred_first_column_hard: bool = False
    prefer_top_down_within_column: bool = True
    enforce_top_down_within_column: bool = False
    enforce_bottom_up_disassembly_within_column: bool = True
    enforce_contiguous_bottom_up_disassembly_chain: bool = True
    enforce_same_a_robot_for_column_disassembly: bool = True
    enforce_robot_column_blocks: bool = False
    column_blocks_by_operation_type: bool = False
    enforce_a_disassembly_priority: bool = False
    enforce_b_inspection_follows_disassembly_completion: bool = False
    preferred_install_column_order: tuple[int, ...] = ()
    enforce_install_start_follows_preferred_order: bool = False
    enforce_alternating_install_by_preferred_order: bool = False
    minimize_initial_start_wait: bool = False
    allow_early_service_start: bool = False
    penalize_column_switch: bool = True
    random_seed: int = 1
    # (before_operation_id, after_operation_id, required_delay)
    additional_precedence_constraints: tuple[tuple[str, str, int], ...] = ()
