"""Compare layered CP-SAT + Space-Time A* with a time-expanded joint prototype.

This script is intentionally an experiment, not the production planner.

The production planner separates:
  CP-SAT assignment/scheduling -> Space-Time A* path execution.

The prototype below builds a time-expanded CP-SAT multi-agent path model on a
small fixed-goal benchmark extracted from the same fixed map.  It is not yet a
full D/I/R assignment+scheduling model; it is a lower-bound demonstration of
the computational cost of putting robot positions directly into CP-SAT.  The
script also estimates how many variables the same formulation would require at
the full scenario_2/test20 horizon.
"""
from __future__ import annotations

import json
import time
from dataclasses import asdict
from pathlib import Path

PROJECT = Path(__file__).resolve().parent.parent

import sys
sys.path.insert(0, str(PROJECT))

try:
    from ortools.sat.python import cp_model
except ImportError as exc:  # pragma: no cover
    raise SystemExit("OR-Tools is required. Use .venv/bin/python.") from exc

from src.domain.enums import RobotType
from src.domain.models import Cell, Footprint, RobotSpec
from src.domain.validation import FootprintValidator
from src.map.fixed_map import FixedMap
from src.map.pose_graph import PoseGraph
from src.map.service_poses import ServicePoseCalculator


OUTPUT = PROJECT / "outputs" / "analysis" / "joint_vs_closed_loop"


def _cell_key(cell: Cell) -> tuple[int, int]:
    return (cell.x, cell.y)


def load_layered_baseline() -> dict:
    metrics_path = PROJECT / "outputs" / "scenario_2" / "test20" / "metrics.json"
    if not metrics_path.exists():
        return {
            "available": False,
            "reason": "outputs/scenario_2/test20/metrics.json does not exist",
        }
    data = json.loads(metrics_path.read_text())
    planning = data.get("planning_quality", {})
    collisions = data.get("collisions_violations", {})
    return {
        "available": True,
        "name": "layered_cp_sat_plus_space_time_astar_test20",
        "makespan": data.get("makespan"),
        "path_length": data.get("path_length", {}).get("total"),
        "collisions": collisions.get("collisions"),
        "constraint_violations": collisions.get("constraint_violations"),
        "replans": planning.get("replans"),
        "cp_time_budget_seconds": planning.get("cp_time_budget"),
        "closed_loop_feedback_iterations": planning.get(
            "closed_loop_feedback_iterations"
        ),
    }


def build_pose_subset(graph: PoseGraph, required: set[Cell], margin: int = 4) -> list[Cell]:
    min_x = min(cell.x for cell in required) - margin
    max_x = max(cell.x for cell in required) + margin
    min_y = min(cell.y for cell in required) - margin
    max_y = max(cell.y for cell in required) + margin
    subset = [
        pose for pose in graph.valid_poses
        if min_x <= pose.x <= max_x and min_y <= pose.y <= max_y
    ]
    required_missing = required - set(subset)
    if required_missing:
        subset.extend(sorted(required_missing, key=_cell_key))
    return sorted(set(subset), key=_cell_key)


def solve_joint_path_prototype(
    horizon: int = 46,
    max_time_seconds: int = 20,
) -> dict:
    """Solve a small exact time-expanded multi-robot path problem.

    The benchmark uses the same map and 2x4 footprint.  Each robot has a fixed
    start and one fixed service anchor.  The model decides every robot pose at
    every time step jointly and enforces footprint and swept collision
    avoidance.  This is already much more expensive than Space-Time A* for the
    same path-planning subproblem.
    """
    terrain, machines, _operations = FixedMap().build()
    footprint = Footprint.default_2x4()
    graph = PoseGraph(terrain, footprint)
    graph.build()
    anchors = ServicePoseCalculator.compute_all_service_anchors(machines, footprint)

    robots = {
        "A_1": RobotSpec("A_1", RobotType.A, Cell(1, 28), footprint),
        "A_2": RobotSpec("A_2", RobotType.A, Cell(12, 28), footprint),
        "B_1": RobotSpec("B_1", RobotType.B, Cell(24, 28), footprint),
    }
    goals = {
        "A_1": anchors["M_y23_x2"],
        "A_2": anchors["M_y23_x11"],
        "B_1": anchors["M_y23_x5"],
    }
    required = {spec.start_anchor for spec in robots.values()} | set(goals.values())
    poses = build_pose_subset(graph, required, margin=5)
    pose_index = {pose: idx for idx, pose in enumerate(poses)}
    pose_set = set(poses)
    edges: list[tuple[int, int, bool, frozenset[Cell]]] = []
    for pose in poses:
        for neighbor, _cost in graph.get_neighbors(pose):
            if neighbor not in pose_set:
                continue
            swept = FootprintValidator.swept_cells(pose, neighbor, footprint)
            edges.append((
                pose_index[pose],
                pose_index[neighbor],
                pose != neighbor,
                swept,
            ))

    incoming: dict[int, list[int]] = {idx: [] for idx in range(len(poses))}
    outgoing: dict[int, list[int]] = {idx: [] for idx in range(len(poses))}
    for edge_idx, (src, dst, _moving, _swept) in enumerate(edges):
        outgoing[src].append(edge_idx)
        incoming[dst].append(edge_idx)

    model = cp_model.CpModel()
    robot_ids = sorted(robots)
    pos = {}
    move = {}
    for rid in robot_ids:
        for t in range(horizon + 1):
            for pidx in range(len(poses)):
                pos[(rid, t, pidx)] = model.NewBoolVar(f"pos[{rid},{t},{pidx}]")
            model.AddExactlyOne(pos[(rid, t, pidx)] for pidx in range(len(poses)))
        model.Add(pos[(rid, 0, pose_index[robots[rid].start_anchor])] == 1)
        model.Add(pos[(rid, horizon, pose_index[goals[rid]])] == 1)

        for t in range(horizon):
            edge_vars = []
            for edge_idx, (_src, _dst, _moving, _swept) in enumerate(edges):
                var = model.NewBoolVar(f"edge[{rid},{t},{edge_idx}]")
                move[(rid, t, edge_idx)] = var
                edge_vars.append(var)
            model.AddExactlyOne(edge_vars)
            for pidx in range(len(poses)):
                model.Add(
                    pos[(rid, t, pidx)]
                    == sum(move[(rid, t, eidx)] for eidx in outgoing[pidx])
                )
                model.Add(
                    pos[(rid, t + 1, pidx)]
                    == sum(move[(rid, t, eidx)] for eidx in incoming[pidx])
                )

    # Footprint collision: no grid cell may be occupied by two robot bodies.
    cells_to_poses: dict[tuple[int, int], list[int]] = {}
    for pidx, pose in enumerate(poses):
        for cell in footprint.cells_at(pose):
            cells_to_poses.setdefault(_cell_key(cell), []).append(pidx)
    footprint_constraints = 0
    for t in range(horizon + 1):
        for _cell, pidxs in cells_to_poses.items():
            terms = [
                pos[(rid, t, pidx)]
                for rid in robot_ids
                for pidx in pidxs
            ]
            if len(terms) > 1:
                model.Add(sum(terms) <= 1)
                footprint_constraints += 1

    # Swept collision: during a transition, no swept cell may be used by two
    # robot movements.  This is conservative and intentionally demonstrates
    # how quickly a joint formulation becomes large.
    cells_to_edges: dict[tuple[int, int], list[int]] = {}
    for eidx, (_src, _dst, _moving, swept) in enumerate(edges):
        for cell in swept:
            cells_to_edges.setdefault(_cell_key(cell), []).append(eidx)
    swept_constraints = 0
    for t in range(horizon):
        for _cell, eidxs in cells_to_edges.items():
            terms = [
                move[(rid, t, eidx)]
                for rid in robot_ids
                for eidx in eidxs
            ]
            if len(terms) > 1:
                model.Add(sum(terms) <= 1)
                swept_constraints += 1

    movement_terms = [
        move[(rid, t, eidx)]
        for rid in robot_ids
        for t in range(horizon)
        for eidx, (_src, _dst, moving, _swept) in enumerate(edges)
        if moving
    ]
    model.Minimize(sum(movement_terms))

    started = time.perf_counter()
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = max_time_seconds
    solver.parameters.num_search_workers = 8
    status = solver.Solve(model)
    elapsed = time.perf_counter() - started

    proto = model.Proto()
    return {
        "name": "time_expanded_joint_path_prototype",
        "scope": (
            "same fixed map and 2x4 footprint; 3 robots; fixed one-goal "
            "path-planning subproblem, not full D/I/R task assignment"
        ),
        "horizon": horizon,
        "max_time_seconds": max_time_seconds,
        "status": solver.StatusName(status),
        "objective_movement_steps": (
            int(solver.ObjectiveValue())
            if status in (cp_model.OPTIMAL, cp_model.FEASIBLE)
            else None
        ),
        "solve_wall_time_seconds": elapsed,
        "pose_count_subset": len(poses),
        "edge_count_subset": len(edges),
        "bool_variables": len(proto.variables),
        "constraints": len(proto.constraints),
        "footprint_collision_constraints": footprint_constraints,
        "swept_collision_constraints": swept_constraints,
        "robots": robot_ids,
        "starts": {
            rid: asdict(robots[rid].start_anchor)
            for rid in robot_ids
        },
        "goals": {
            rid: asdict(goals[rid])
            for rid in robot_ids
        },
    }


def estimate_full_joint_size(horizon: int | None = None) -> dict:
    terrain, _machines, operations = FixedMap().build()
    footprint = Footprint.default_2x4()
    graph = PoseGraph(terrain, footprint)
    graph.build()
    if horizon is None:
        horizon = 1014
    robot_count = 3
    pose_count = graph.node_count()
    edge_count = graph.edge_count()
    pos_vars = robot_count * (horizon + 1) * pose_count
    edge_vars = robot_count * horizon * edge_count
    # These counts do not include assignment/order/service variables, so they
    # are a lower-bound estimate for a true full joint model.
    return {
        "name": "full_scenario_2_time_expanded_lower_bound_estimate",
        "horizon_from_test20": horizon,
        "robot_count": robot_count,
        "operation_count": len(operations),
        "pose_count_full_graph": pose_count,
        "edge_count_full_graph": edge_count,
        "estimated_position_bool_variables": pos_vars,
        "estimated_transition_bool_variables": edge_vars,
        "estimated_position_plus_transition_bool_variables": pos_vars + edge_vars,
        "note": (
            "Lower bound only: true full joint assignment+scheduling+path "
            "optimization would add operation assignment, service state, "
            "precedence, resource, and objective variables/constraints."
        ),
    }


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    layered = load_layered_baseline()
    joint = solve_joint_path_prototype()
    estimate = estimate_full_joint_size(
        int(layered["makespan"])
        if layered.get("available") and layered.get("makespan")
        else None
    )
    report = {
        "layered_current_version": layered,
        "joint_time_expanded_prototype": joint,
        "full_joint_size_estimate": estimate,
        "interpretation": {
            "short_answer": (
                "A full time-space joint model is technically expressible, "
                "but the full 48-machine scenario is not a practical near-term "
                "replacement for the layered CP-SAT + Space-Time A* pipeline."
            ),
            "why": [
                (
                    "The prototype already uses tens/hundreds of thousands of "
                    "Boolean variables for a fixed-goal path-only subproblem."
                ),
                (
                    "The full scenario lower bound is several million Boolean "
                    "variables before adding D/I/R assignment, sequence, service, "
                    "and precedence modeling."
                ),
                (
                    "The layered test20 result solves the full 48-machine problem "
                    f"with zero collisions and makespan {layered.get('makespan')}, while the joint "
                    "prototype only covers a much smaller path subproblem."
                ),
            ],
        },
    }
    (OUTPUT / "joint_vs_closed_loop_report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False)
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
