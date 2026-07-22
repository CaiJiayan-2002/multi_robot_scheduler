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
    enforce_same_b_robot_for_column_inspection: bool = False
    enforce_inspection_after_full_column_disassembly: bool = False
    enforce_contiguous_bottom_up_inspection_chain: bool = False
    enforce_robot_column_blocks: bool = False
    column_blocks_by_operation_type: bool = False
    enforce_a_disassembly_priority: bool = False
    enforce_b_inspection_follows_disassembly_completion: bool = False
    disable_runtime_b_inspection_reorder: bool = False
    # Physical x-column groups that must advance left-to-right for each operation type.
    # Example for the fixed 8-column map: ((2, 5), (8, 11), (14, 17), (20, 23)).
    # Within one wave the solver is still free to optimize assignment, sequence and timing.
    column_wave_order: tuple[tuple[int, ...], ...] = ()
    enforce_disassembly_column_wave_order: bool = False
    enforce_inspection_column_wave_order: bool = False
    enforce_install_column_wave_order: bool = False
    # Physical x-column order for B inspection.  This can encode a soft-looking
    # production preference as a hard CP-SAT ordering constraint while still
    # leaving operation timing and inter-robot pipeline gaps to the solver.
    preferred_inspection_column_order: tuple[int, ...] = ()
    enforce_inspection_start_follows_preferred_order: bool = False
    enforce_inspection_finish_column_before_next: bool = False
    preferred_disassembly_column_order: tuple[int, ...] = ()
    enforce_alternating_disassembly_by_preferred_order: bool = False
    preferred_install_column_order: tuple[int, ...] = ()
    enforce_install_start_follows_preferred_order: bool = False
    enforce_alternating_install_by_preferred_order: bool = False
    minimize_initial_start_wait: bool = False
    allow_early_service_start: bool = False
    penalize_column_switch: bool = True
    random_seed: int = 1
    num_search_workers: int = 8
    # (before_operation_id, after_operation_id, required_delay)
    additional_precedence_constraints: tuple[tuple[str, str, int], ...] = ()
