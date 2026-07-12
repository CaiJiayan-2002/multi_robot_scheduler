"""OR-Tools CP-SAT 完整任务分配与调度模型。"""
from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any

from ..domain.enums import OperationType, RobotType, ResultStatus
from ..domain.models import ScheduleResult, SchedulingProblem
from .config import SolverConfig

try:
    from ortools.sat.python import cp_model
    _ORTOOLS_AVAILABLE = True
except ImportError:  # pragma: no cover - 正式入口会明确报错
    cp_model = None
    _ORTOOLS_AVAILABLE = False


@dataclass
class ModelArtifacts:
    operations: list[str]
    eligible_by_robot: dict[str, list[str]]
    assigned: dict[tuple[str, str], Any]
    start: dict[str, Any]
    end: dict[str, Any]
    intervals: dict[tuple[str, str], Any]
    arcs: dict[tuple[str, str, str], Any]
    makespan: Any
    total_travel: Any
    column_switches: Any
    load_gap: Any
    preference_penalty: Any


class CpSatScheduler:
    """同时决定分配、顺序、时间和静态旅行衔接的正式求解器。"""

    def __init__(self, config: SolverConfig | None = None) -> None:
        if not _ORTOOLS_AVAILABLE:
            raise ImportError(
                "OR-Tools 未安装；正式模式禁止 fallback。请安装 requirements.txt。"
            )
        self.config = config or SolverConfig()
        self._model: cp_model.CpModel | None = None
        self._solver: cp_model.CpSolver | None = None
        self._artifacts: ModelArtifacts | None = None

    def solve(
        self,
        problem: SchedulingProblem,
        max_time_seconds: int | None = None,
        mode: str = "assignment_schedule",
    ) -> ScheduleResult:
        solve_started = time.perf_counter()
        if mode != "assignment_schedule":
            return ScheduleResult(
                status=ResultStatus.INVALID_INPUT.value,
                solver_backend="ortools_cp_sat",
                solver_mode=mode,
                solver_status="INVALID_MODE",
                fallback_used=False,
                operation_sequence_source="none",
                fallback_reason=f"unsupported formal mode: {mode}",
            )
        if not problem.travel_times:
            raise ValueError("assignment_schedule requires footprint-aware travel_times")

        self._model, self._artifacts = self._build_model(problem)
        total_budget = float(max_time_seconds or self.config.max_time_seconds)
        def new_solver(seconds: float):
            solver = cp_model.CpSolver()
            solver.parameters.max_time_in_seconds = max(1.0, seconds)
            solver.parameters.num_search_workers = 8
            solver.parameters.random_seed = self.config.random_seed
            return solver

        # Phase 1: makespan
        self._model.Minimize(self._artifacts.makespan)
        self._solver = new_solver(total_budget * 0.5)
        status = self._solver.Solve(self._model)
        if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            return self._failure_result(status)
        best_makespan = self._solver.Value(self._artifacts.makespan)
        self._model.Add(
            self._artifacts.makespan
            <= best_makespan + self.config.makespan_tolerance
        )

        # Phase 2: total static-A* travel
        self._model.Minimize(self._artifacts.total_travel)
        phase2_solver = new_solver(total_budget * 0.3)
        status2 = phase2_solver.Solve(self._model)
        if status2 in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            status = status2
            self._solver = phase2_solver
            best_travel = self._solver.Value(self._artifacts.total_travel)
            self._model.Add(self._artifacts.total_travel <= best_travel)

        # Phase 3: column switches, load gap, preferences and early starts.
        secondary = (
            self._artifacts.column_switches * 100_000
            + self._artifacts.load_gap * 1_000
            + self._artifacts.preference_penalty * 100
            + sum(self._artifacts.start.values())
        )
        self._model.Minimize(secondary)
        phase3_solver = new_solver(total_budget * 0.2)
        status3 = phase3_solver.Solve(self._model)
        if status3 in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            status = status3
            self._solver = phase3_solver

        from .schedule_extractor import extract_cp_sat_schedule
        result = extract_cp_sat_schedule(
            problem, self.config, self._solver, status, self._artifacts
        )
        result.solve_time_seconds = time.perf_counter() - solve_started
        return result

    def _build_model(
        self, problem: SchedulingProblem
    ) -> tuple[cp_model.CpModel, ModelArtifacts]:
        model = cp_model.CpModel()
        # sorted 只用于稳定变量编号；求解结果顺序完全由 arc 决策产生。
        operation_ids = sorted(problem.operations)
        robot_ids = sorted(problem.robots)
        horizon = sum(op.duration for op in problem.operations.values())
        horizon += max(problem.travel_times.values(), default=0) * len(operation_ids)

        start = {
            op_id: model.NewIntVar(0, horizon, f"start[{op_id}]")
            for op_id in operation_ids
        }
        end = {
            op_id: model.NewIntVar(0, horizon, f"end[{op_id}]")
            for op_id in operation_ids
        }
        assigned: dict[tuple[str, str], Any] = {}
        intervals: dict[tuple[str, str], Any] = {}
        eligible_by_robot = {rid: [] for rid in robot_ids}

        for op_id in operation_ids:
            op = problem.operations[op_id]
            model.Add(end[op_id] == start[op_id] + op.duration)
            eligible = [
                rid for rid in robot_ids
                if problem.robots[rid].robot_type == op.eligible_robot_type
            ]
            if not eligible:
                raise ValueError(f"operation {op_id} has no eligible robot")
            for rid in eligible:
                presence = model.NewBoolVar(f"assigned[{op_id},{rid}]")
                assigned[(op_id, rid)] = presence
                eligible_by_robot[rid].append(op_id)
                intervals[(op_id, rid)] = model.NewOptionalIntervalVar(
                    start[op_id], op.duration, end[op_id], presence,
                    f"optional_interval[{op_id},{rid}]",
                )
            model.AddExactlyOne(assigned[(op_id, rid)] for rid in eligible)

        if self.config.require_same_a_robot_for_disassemble_and_install:
            for mid in problem.machines:
                d_id, r_id = f"{mid}_D", f"{mid}_R"
                for rid, robot in problem.robots.items():
                    if robot.robot_type == RobotType.A:
                        model.Add(assigned[(d_id, rid)] == assigned[(r_id, rid)])

        if self.config.enforce_same_a_robot_for_column_disassembly:
            columns: dict[int, list[str]] = {}
            for mid, machine in problem.machines.items():
                columns.setdefault(machine.cells[0].x, []).append(mid)
            a_robot_ids = [
                rid for rid, robot in problem.robots.items()
                if robot.robot_type == RobotType.A
            ]
            for machine_ids in columns.values():
                if len(machine_ids) < 2:
                    continue
                anchor_op = f"{machine_ids[0]}_D"
                for mid in machine_ids[1:]:
                    op_id = f"{mid}_D"
                    for rid in a_robot_ids:
                        # 列是自然的拆机工作包；具体哪台 A 负责该列仍由
                        # CP-SAT 决定，但不能把同一列拆机切碎给多台 A，
                        # 否则会出现一台机器人沿列移动时跳过中间机器。
                        model.Add(assigned[(op_id, rid)] == assigned[(anchor_op, rid)])

        if self.config.enforce_top_down_within_column:
            columns: dict[int, list[str]] = {}
            for mid, machine in problem.machines.items():
                columns.setdefault(machine.cells[0].x, []).append(mid)
            for machine_ids in columns.values():
                by_row = sorted(machine_ids, key=lambda mid: problem.machines[mid].row)
                for upper, lower in zip(by_row, by_row[1:]):
                    upper_op, lower_op = f"{upper}_D", f"{lower}_D"
                    for rid, robot in problem.robots.items():
                        if robot.robot_type == RobotType.A:
                            model.Add(end[upper_op] <= start[lower_op]).OnlyEnforceIf([
                                assigned[(upper_op, rid)], assigned[(lower_op, rid)]
                            ])

        if self.config.enforce_bottom_up_disassembly_within_column:
            columns: dict[int, list[str]] = {}
            for mid, machine in problem.machines.items():
                columns.setdefault(machine.cells[0].x, []).append(mid)
            for machine_ids in columns.values():
                # 主干道在地图下方；拆机机器人从下方进入列时，正式
                # 模式要求同一台 A 在同一列内按 y=23,19,...,3 连续拆机，
                # 防止出现 y=23→19→15→7→3→11 这种跳过中间离心机再
                # 回来补做的规划。
                by_row_from_bottom = sorted(
                    machine_ids,
                    key=lambda mid: problem.machines[mid].row,
                    reverse=True,
                )
                for lower, upper in zip(by_row_from_bottom, by_row_from_bottom[1:]):
                    lower_op, upper_op = f"{lower}_D", f"{upper}_D"
                    for rid, robot in problem.robots.items():
                        if robot.robot_type == RobotType.A:
                            model.Add(end[lower_op] <= start[upper_op]).OnlyEnforceIf([
                                assigned[(lower_op, rid)], assigned[(upper_op, rid)]
                            ])

        for mid in problem.machines:
            d_id, i_id, r_id = f"{mid}_D", f"{mid}_I", f"{mid}_R"
            model.Add(end[d_id] <= start[i_id])
            model.Add(end[i_id] <= start[r_id])
        for before, after, delay in self.config.additional_precedence_constraints:
            if before not in end or after not in start:
                raise ValueError(f"unknown repair precedence: {before} -> {after}")
            model.Add(start[after] >= end[before] + delay)

        for rid in robot_ids:
            model.AddNoOverlap(
                intervals[(op_id, rid)] for op_id in eligible_by_robot[rid]
            )

        arcs: dict[tuple[str, str, str], Any] = {}
        travel_terms = []
        switch_terms = []
        preference_terms = []
        unique_columns = sorted({m.cells[0].x for m in problem.machines.values()})
        preferred_x = None
        if self.config.preferred_first_column is not None:
            idx = self.config.preferred_first_column - 1
            if 0 <= idx < len(unique_columns):
                preferred_x = unique_columns[idx]

        for rid in robot_ids:
            ops = eligible_by_robot[rid]
            node = {op_id: index + 1 for index, op_id in enumerate(ops)}
            circuit_arcs: list[tuple[int, int, Any]] = []
            empty = model.NewBoolVar(f"arc[{rid},START,END]")
            arcs[(rid, "START", "END")] = empty
            circuit_arcs.append((0, 0, empty))
            assign_sum = sum(assigned[(op_id, rid)] for op_id in ops)
            model.Add(assign_sum == 0).OnlyEnforceIf(empty)
            model.Add(assign_sum >= 1).OnlyEnforceIf(empty.Not())

            for op_id in ops:
                self_loop = assigned[(op_id, rid)].Not()
                circuit_arcs.append((node[op_id], node[op_id], self_loop))

                first = model.NewBoolVar(f"arc[{rid},START,{op_id}]")
                last = model.NewBoolVar(f"arc[{rid},{op_id},END]")
                arcs[(rid, "START", op_id)] = first
                arcs[(rid, op_id, "END")] = last
                circuit_arcs.extend(((0, node[op_id], first), (node[op_id], 0, last)))
                model.AddImplication(first, assigned[(op_id, rid)])
                model.AddImplication(last, assigned[(op_id, rid)])
                initial_t = problem.travel_times[(rid, "START", op_id)]
                model.Add(start[op_id] >= initial_t).OnlyEnforceIf(first)
                travel_terms.extend((first * initial_t, last * problem.travel_times[(rid, op_id, "END")]))

                op = problem.operations[op_id]
                first_is_preferred = (
                    problem.robots[rid].robot_type == RobotType.A
                    and op.operation_type == OperationType.DISASSEMBLE
                    and preferred_x is not None
                    and problem.machines[op.machine_id].cells[0].x == preferred_x
                )
                if self.config.preferred_first_column_hard and preferred_x is not None:
                    if problem.robots[rid].robot_type == RobotType.A and not first_is_preferred:
                        model.Add(first == 0)
                elif (
                    preferred_x is not None
                    and problem.robots[rid].robot_type == RobotType.A
                    and not first_is_preferred
                ):
                    preference_terms.append(first)

            for from_id in ops:
                from_machine = problem.machines[problem.operations[from_id].machine_id]
                for to_id in ops:
                    if from_id == to_id:
                        continue
                    arc = model.NewBoolVar(f"arc[{rid},{from_id},{to_id}]")
                    arcs[(rid, from_id, to_id)] = arc
                    circuit_arcs.append((node[from_id], node[to_id], arc))
                    model.AddImplication(arc, assigned[(from_id, rid)])
                    model.AddImplication(arc, assigned[(to_id, rid)])
                    travel = problem.travel_times[(rid, from_id, to_id)]
                    model.Add(start[to_id] >= end[from_id] + travel).OnlyEnforceIf(arc)
                    travel_terms.append(arc * travel)
                    to_machine = problem.machines[problem.operations[to_id].machine_id]
                    if (
                        self.config.penalize_column_switch
                        and from_machine.cells[0].x != to_machine.cells[0].x
                    ):
                        switch_terms.append(arc)
                    if self.config.prefer_top_down_within_column:
                        from_op, to_op = problem.operations[from_id], problem.operations[to_id]
                        wrong_way = (
                            from_op.operation_type == OperationType.DISASSEMBLE
                            and to_op.operation_type == OperationType.DISASSEMBLE
                            and from_machine.cells[0].x == to_machine.cells[0].x
                            and from_machine.row > to_machine.row
                        )
                        if wrong_way:
                            if self.config.enforce_top_down_within_column:
                                model.Add(arc == 0)
                            else:
                                preference_terms.append(arc)
            model.AddCircuit(circuit_arcs)

        total_travel = model.NewIntVar(0, horizon * max(1, len(robot_ids)), "total_travel")
        model.Add(total_travel == sum(travel_terms))
        column_switches = model.NewIntVar(0, len(operation_ids), "column_switch_count")
        model.Add(column_switches == (sum(switch_terms) if switch_terms else 0))
        preference_penalty = model.NewIntVar(0, len(operation_ids) * 2, "preference_penalty")
        model.Add(preference_penalty == (sum(preference_terms) if preference_terms else 0))

        load_gaps = []
        for robot_type in (RobotType.A, RobotType.B):
            typed = [rid for rid in robot_ids if problem.robots[rid].robot_type == robot_type]
            if len(typed) < 2:
                continue
            loads = []
            for rid in typed:
                load = model.NewIntVar(0, horizon, f"service_load[{rid}]")
                model.Add(load == sum(
                    problem.operations[op_id].duration * assigned[(op_id, rid)]
                    for op_id in eligible_by_robot[rid]
                ))
                loads.append(load)
            maximum = model.NewIntVar(0, horizon, f"max_load[{robot_type.value}]")
            minimum = model.NewIntVar(0, horizon, f"min_load[{robot_type.value}]")
            gap = model.NewIntVar(0, horizon, f"load_gap[{robot_type.value}]")
            model.AddMaxEquality(maximum, loads)
            model.AddMinEquality(minimum, loads)
            model.Add(gap == maximum - minimum)
            load_gaps.append(gap)
        load_gap = model.NewIntVar(0, horizon, "load_gap")
        model.Add(load_gap == (sum(load_gaps) if load_gaps else 0))

        install_ends = [
            end[op_id] for op_id in operation_ids
            if problem.operations[op_id].operation_type == OperationType.INSTALL
        ]
        makespan = model.NewIntVar(0, horizon, "makespan")
        model.AddMaxEquality(makespan, install_ends)
        artifacts = ModelArtifacts(
            operation_ids, eligible_by_robot, assigned, start, end, intervals,
            arcs, makespan, total_travel, column_switches, load_gap,
            preference_penalty,
        )
        if not self._add_single_pair_warm_start(model, problem, artifacts):
            self._add_multi_robot_warm_start(model, problem, artifacts)
        return model, artifacts

    @staticmethod
    def _add_single_pair_warm_start(model, problem, artifacts) -> bool:
        """为 1A1B 提供基于 A* 距离的可行初值，不固定任何决策。"""
        a_ids = [rid for rid, r in problem.robots.items() if r.robot_type == RobotType.A]
        b_ids = [rid for rid, r in problem.robots.items() if r.robot_type == RobotType.B]
        if len(a_ids) != 1 or len(b_ids) != 1:
            return False
        a_id, b_id = a_ids[0], b_ids[0]
        remaining = set(problem.machines)
        machine_order = []
        previous = "START"
        while remaining:
            chosen = min(
                remaining,
                key=lambda mid: (
                    problem.travel_times[(
                        a_id, previous,
                        f"{mid}_D",
                    )],
                    mid,
                ),
            )
            machine_order.append(chosen)
            remaining.remove(chosen)
            previous = f"{chosen}_D"

        routes = {
            a_id: [f"{mid}_D" for mid in machine_order] + [f"{mid}_R" for mid in machine_order],
            b_id: [f"{mid}_I" for mid in machine_order],
        }
        for (op_id, rid), var in artifacts.assigned.items():
            model.AddHint(var, int(op_id in routes[rid]))
        for key, var in artifacts.arcs.items():
            rid, source, target = key
            route = routes[rid]
            selected = False
            if route:
                selected = (
                    (source == "START" and target == route[0])
                    or (source == route[-1] and target == "END")
                    or any(source == route[i] and target == route[i + 1] for i in range(len(route) - 1))
                )
            else:
                selected = source == "START" and target == "END"
            model.AddHint(var, int(selected))

        times = {}
        clock = 0
        previous = "START"
        for op_id in routes[a_id][:len(machine_order)]:
            clock += problem.travel_times[(a_id, previous, op_id)]
            times[op_id] = (clock, clock + problem.operations[op_id].duration)
            clock = times[op_id][1]
            previous = op_id
        b_clock, previous = 0, "START"
        for mid in machine_order:
            op_id = f"{mid}_I"
            b_clock += problem.travel_times[(b_id, previous, op_id)]
            b_clock = max(b_clock, times[f"{mid}_D"][1])
            times[op_id] = (b_clock, b_clock + problem.operations[op_id].duration)
            b_clock = times[op_id][1]
            previous = op_id
        previous = routes[a_id][len(machine_order) - 1]
        for mid in machine_order:
            op_id = f"{mid}_R"
            clock += problem.travel_times[(a_id, previous, op_id)]
            clock = max(clock, times[f"{mid}_I"][1])
            times[op_id] = (clock, clock + problem.operations[op_id].duration)
            clock = times[op_id][1]
            previous = op_id
        for op_id, (start, end) in times.items():
            model.AddHint(artifacts.start[op_id], start)
            model.AddHint(artifacts.end[op_id], end)
        return True

    @staticmethod
    def _add_multi_robot_warm_start(model, problem, artifacts) -> None:
        """多机器人自动距离/负载贪心仅作为 CP-SAT 初值。"""
        typed = {
            RobotType.A: sorted(rid for rid, r in problem.robots.items() if r.robot_type == RobotType.A),
            RobotType.B: sorted(rid for rid, r in problem.robots.items() if r.robot_type == RobotType.B),
        }
        assignment: dict[str, str] = {}
        loads = {rid: 0 for rid in problem.robots}
        # 以静态 A* 起点距离和当前服务负载选择 hint 分配，不固定模型。
        for mid in problem.machines:
            for suffix, robot_type in (("D", RobotType.A), ("I", RobotType.B), ("R", RobotType.A)):
                op_id = f"{mid}_{suffix}"
                if suffix == "R" and mid + "_D" in assignment:
                    candidates = typed[robot_type]
                else:
                    candidates = typed[robot_type]
                rid = min(
                    candidates,
                    key=lambda candidate: (
                        loads[candidate] * 10
                        + problem.travel_times[(candidate, "START", op_id)],
                        candidate,
                    ),
                )
                assignment[op_id] = rid
                loads[rid] += problem.operations[op_id].duration

        def nearest_route(rid: str, operation_ids: list[str]) -> list[str]:
            route, remaining, previous = [], set(operation_ids), "START"
            while remaining:
                chosen = min(
                    remaining,
                    key=lambda op_id: (
                        problem.travel_times[(rid, previous, op_id)], op_id
                    ),
                )
                route.append(chosen)
                remaining.remove(chosen)
                previous = chosen
            return route

        routes = {}
        for rid, robot in problem.robots.items():
            own = [op for op, owner in assignment.items() if owner == rid]
            if robot.robot_type == RobotType.A:
                d_ops = [op for op in own if op.endswith("_D")]
                r_ops = [op for op in own if op.endswith("_R")]
                routes[rid] = nearest_route(rid, d_ops) + nearest_route(rid, r_ops)
            else:
                routes[rid] = nearest_route(rid, own)

        for (op_id, rid), var in artifacts.assigned.items():
            model.AddHint(var, int(assignment[op_id] == rid))
        for (rid, source, target), var in artifacts.arcs.items():
            route = routes[rid]
            selected = (
                (not route and source == "START" and target == "END")
                or (route and source == "START" and target == route[0])
                or (route and source == route[-1] and target == "END")
                or any(source == route[i] and target == route[i + 1] for i in range(len(route) - 1))
            )
            model.AddHint(var, int(selected))

        # 建立 hint 路由边 + 工艺边的 DAG，计算一致的最早时间。
        predecessors = {op_id: [] for op_id in problem.operations}
        initial = {op_id: 0 for op_id in problem.operations}
        for rid, route in routes.items():
            previous = "START"
            for op_id in route:
                if previous == "START":
                    initial[op_id] = problem.travel_times[(rid, "START", op_id)]
                else:
                    predecessors[op_id].append((
                        previous, problem.travel_times[(rid, previous, op_id)]
                    ))
                previous = op_id
        for mid in problem.machines:
            predecessors[f"{mid}_I"].append((f"{mid}_D", 0))
            predecessors[f"{mid}_R"].append((f"{mid}_I", 0))

        unresolved = set(problem.operations)
        times = {}
        while unresolved:
            ready = [
                op_id for op_id in unresolved
                if all(pred in times for pred, _ in predecessors[op_id])
            ]
            if not ready:
                return  # hint 含环时宁可不给时间提示，也不影响正式模型。
            for op_id in ready:
                earliest = initial[op_id]
                for pred, travel in predecessors[op_id]:
                    earliest = max(earliest, times[pred][1] + travel)
                times[op_id] = (
                    earliest, earliest + problem.operations[op_id].duration
                )
                unresolved.remove(op_id)
        for op_id, (start, end) in times.items():
            model.AddHint(artifacts.start[op_id], start)
            model.AddHint(artifacts.end[op_id], end)

    def _failure_result(self, status: int) -> ScheduleResult:
        name = self._solver.StatusName(status) if self._solver else str(status)
        return ScheduleResult(
            status=ResultStatus.INFEASIBLE.value,
            solver_backend="ortools_cp_sat",
            solver_mode="assignment_schedule",
            solver_status=name,
            fallback_used=False,
            fallback_reason=f"CP-SAT returned {name}; fallback disabled",
            operation_sequence_source="none",
        )
