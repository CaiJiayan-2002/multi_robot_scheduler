from __future__ import annotations

import pytest

pytest.importorskip("ortools")

from src.domain.enums import RobotType
from src.domain.models import Cell, RobotSpec, SchedulingProblem, Footprint
from src.map.fixed_map import FixedMap
from src.map.pose_graph import PoseGraph
from src.solver.cp_sat_model import CpSatScheduler
from src.solver.travel_time import build_operation_travel_times
from src.solver.config import SolverConfig
from src.solver.scheduler import solve_assignment_schedule


@pytest.fixture(scope="module")
def formal_result():
    terrain, all_machines, all_operations = FixedMap().build()
    machine_ids = list(all_machines)[:6]
    machines = {mid: all_machines[mid] for mid in machine_ids}
    operations = {
        op_id: op for op_id, op in all_operations.items()
        if op.machine_id in machines
    }
    robots = {
        "A_1": RobotSpec("A_1", RobotType.A, Cell(1, 28)),
        "B_1": RobotSpec("B_1", RobotType.B, Cell(24, 28)),
    }
    result = solve_assignment_schedule(
        terrain, machines, operations, robots,
        SolverConfig(max_time_seconds=6, preferred_first_column=None),
    )
    return result, machines, operations, robots


def test_formal_mode_never_falls_back(formal_result):
    result, *_ = formal_result
    assert result.fallback_used is False
    assert result.solver_backend == "ortools_cp_sat"
    assert result.solver_mode == "assignment_schedule"
    assert result.operation_sequence_source == "cp_sat"


def test_all_operations_assigned_once_with_times_and_types(formal_result):
    result, _, operations, robots = formal_result
    assert len(result.assignments) == len(operations)
    assert len({item["operation_id"] for item in result.assignments}) == len(operations)
    for item in result.assignments:
        op = operations[item["operation_id"]]
        assert robots[item["assigned_robot_id"]].robot_type == op.eligible_robot_type
        assert item["planned_end_time"] - item["planned_start_time"] == op.duration
        assert item["arc_variable_name"].startswith("arc[")


def test_precedence_no_overlap_travel_and_sequence(formal_result):
    result, machines, operations, _ = formal_result
    by_id = {item["operation_id"]: item for item in result.assignments}
    for mid in machines:
        assert by_id[f"{mid}_D"]["planned_end_time"] <= by_id[f"{mid}_I"]["planned_start_time"]
        assert by_id[f"{mid}_I"]["planned_end_time"] <= by_id[f"{mid}_R"]["planned_start_time"]
    for schedule in result.robot_schedules.values():
        details = schedule.ordered_operations
        assert [d["sequence_index"] for d in details] == list(range(len(details)))
        assert len({d["sequence_index"] for d in details}) == len(details)
        for previous, current in zip(details, details[1:]):
            assert previous["planned_end_time"] <= current["planned_start_time"]
            assert (
                current["planned_start_time"]
                >= previous["planned_end_time"] + current["travel_time_from_predecessor"]
            )
            assert current["predecessor_operation_id"] == previous["operation_id"]


@pytest.mark.parametrize("a_count,b_count", [(2, 1), (4, 2)])
def test_multi_robot_formal_schedule_is_feasible(a_count, b_count):
    terrain, all_machines, all_operations = FixedMap().build()
    machine_ids = list(all_machines)[:8]
    machines = {mid: all_machines[mid] for mid in machine_ids}
    operations = {k: v for k, v in all_operations.items() if v.machine_id in machines}
    robots = {}
    for i in range(a_count):
        robots[f"A_{i+1}"] = RobotSpec(f"A_{i+1}", RobotType.A, Cell(1 + i * 4, 28))
    for i in range(b_count):
        robots[f"B_{i+1}"] = RobotSpec(
            f"B_{i+1}", RobotType.B, Cell(17 + i * 4, 28)
        )
    result = solve_assignment_schedule(
        terrain, machines, operations, robots,
        SolverConfig(max_time_seconds=8, preferred_first_column=None),
    )
    assert result.solver_status in ("FEASIBLE", "OPTIMAL")
    assert len(result.assignments) == len(operations)
    assert result.fallback_used is False
    assert all(s.ordered_operations for s in result.robot_schedules.values())


def test_same_a_robot_constraint_is_configurable():
    terrain, all_machines, all_operations = FixedMap().build()
    machine_ids = list(all_machines)[:3]
    machines = {mid: all_machines[mid] for mid in machine_ids}
    operations = {k: v for k, v in all_operations.items() if v.machine_id in machines}
    robots = {
        "A_1": RobotSpec("A_1", RobotType.A, Cell(1, 28)),
        "A_2": RobotSpec("A_2", RobotType.A, Cell(12, 28)),
        "B_1": RobotSpec("B_1", RobotType.B, Cell(24, 28)),
    }
    result = solve_assignment_schedule(
        terrain, machines, operations, robots,
        SolverConfig(
            max_time_seconds=5,
            preferred_first_column=None,
            require_same_a_robot_for_disassemble_and_install=True,
        ),
    )
    owner = {item["operation_id"]: item["assigned_robot_id"] for item in result.assignments}
    for mid in machines:
        assert owner[f"{mid}_D"] == owner[f"{mid}_R"]


def test_changed_travel_matrix_can_change_cp_sat_sequence():
    terrain, all_machines, all_operations = FixedMap().build()
    machine_ids = list(all_machines)[:4]
    machines = {mid: all_machines[mid] for mid in machine_ids}
    operations = {k: v for k, v in all_operations.items() if v.machine_id in machines}

    robots = {
        "A_1": RobotSpec("A_1", RobotType.A, Cell(1, 28)),
        "B_1": RobotSpec("B_1", RobotType.B, Cell(24, 28)),
    }
    graph = PoseGraph(terrain, Footprint.default_2x4())
    graph.build()
    base = build_operation_travel_times(
        graph, Footprint.default_2x4(), operations, robots
    )

    def solve(preferred_mid):
        travel = dict(base)
        for op_id, op in operations.items():
            if op.operation_type.value == "DISASSEMBLE":
                travel[("A_1", "START", op_id)] = (
                    0 if op.machine_id == preferred_mid else 1000
                )
        problem = SchedulingProblem(machines, operations, robots, travel)
        return CpSatScheduler(SolverConfig(
            max_time_seconds=5,
            preferred_first_column=None,
            penalize_column_switch=False,
        )).solve(problem)

    left_mid, right_mid = machine_ids[0], machine_ids[-1]
    left_first = solve(left_mid).robot_schedules["A_1"].first_operation
    right_first = solve(right_mid).robot_schedules["A_1"].first_operation
    assert left_first == f"{left_mid}_D"
    assert right_first == f"{right_mid}_D"
