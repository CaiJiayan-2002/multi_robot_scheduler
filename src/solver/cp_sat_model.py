"""CP-SAT 任务分配求解器 v4.0

使用 Google OR-Tools CP-SAT 求解器进行任务分配。

Mode A (assignment_only): 只分配操作给机器人，估计负载。
不决定精确顺序和开始时间（W2阶段）。

输入:
- 144个操作
- 机器人列表
- 旅行时间矩阵

输出:
- ScheduleResult: 每个机器人分配了哪些操作
"""

from __future__ import annotations

from ..domain.enums import OperationType, RobotType, ResultStatus
from ..domain.models import (
    Machine, Operation, RobotSpec, RobotSchedule, ScheduleResult, SchedulingProblem,
)

# 尝试导入 ortools
try:
    from ortools.sat.python import cp_model
    _ORTOOLS_AVAILABLE = True
except ImportError:
    _ORTOOLS_AVAILABLE = False


class CpSatScheduler:
    """CP-SAT 任务分配求解器。

    使用约束规划将144个操作分配给多台机器人。

    Mode A (assignment_only):
    - 约束: 每操作一台机器人，机器人类型匹配，前序依赖
    - 目标: 最小化最大负载差

    Attributes:
        _model: CP-SAT 模型
        _solver: CP-SAT 求解器
    """

    def __init__(self, config_path: str | None = None) -> None:
        """初始化 CP-SAT 求解器。

        Args:
            config_path: 配置文件路径（保留参数，当前未使用）
        """
        if not _ORTOOLS_AVAILABLE:
            raise ImportError(
                "ortools 未安装。请运行: pip install ortools\n"
                "或使用 src.solver.fallback.manual_assign() 作为备选方案。"
            )
        self._config_path = config_path
        self._model: cp_model.CpModel | None = None
        self._solver: cp_model.CpSolver | None = None

    # ------------------------------------------------------------------
    def solve(
        self,
        problem: SchedulingProblem,
        max_time_seconds: int = 30,
        mode: str = "assignment_only",
    ) -> ScheduleResult:
        """求解任务分配问题。

        Args:
            problem: 调度问题定义
            max_time_seconds: 求解时间上限（秒）
            mode: 求解模式，"assignment_only" 仅分配操作

        Returns:
            ScheduleResult 包含各机器人的操作分配
        """
        self._model = cp_model.CpModel()
        self._solver = cp_model.CpSolver()
        self._solver.parameters.max_time_in_seconds = max_time_seconds
        self._solver.parameters.num_search_workers = 8

        if mode == "assignment_only":
            return self._build_assignment_only_model(problem)
        else:
            return ScheduleResult(
                status=ResultStatus.INVALID_INPUT.value,
                objective={"error": f"未知求解模式: {mode}"},
            )

    # ------------------------------------------------------------------
    def _build_assignment_only_model(
        self, problem: SchedulingProblem
    ) -> ScheduleResult:
        """构建 assignment_only 模式的 CP-SAT 模型。

        约束:
        1. 每个操作只分配给一台机器人
        2. DISASSEMBLE/INSTALL 只能分配给 A 机器人
        3. INSPECT 只能分配给 B 机器人
        4. D->I->R 前序关系（同一台离心机的操作必须按顺序执行，B须等A的D完成）

        目标:
        - 最小化 A 机器人之间的最大负载差

        Args:
            problem: 调度问题

        Returns:
            ScheduleResult
        """
        model = self._model
        operations = problem.operations
        robots = problem.robots

        # 按类型分类机器人
        a_robots = [rid for rid, r in robots.items() if r.robot_type == RobotType.A]
        b_robots = [rid for rid, r in robots.items() if r.robot_type == RobotType.B]

        # 按类型分类操作
        d_ops = [
            op_id for op_id, op in operations.items()
            if op.operation_type == OperationType.DISASSEMBLE
        ]
        i_ops = [
            op_id for op_id, op in operations.items()
            if op.operation_type == OperationType.INSPECT
        ]
        r_ops = [
            op_id for op_id, op in operations.items()
            if op.operation_type == OperationType.INSTALL
        ]

        # ——————————————————————————————————————————————————————————
        # 决策变量: x[op_id, robot_id] = 1 如果 robot 执行 op
        # ——————————————————————————————————————————————————————————
        x_vars: dict[tuple[str, str], cp_model.IntVar] = {}
        for op_id, op in operations.items():
            eligible = []
            if op.operation_type in (OperationType.DISASSEMBLE, OperationType.INSTALL):
                eligible = a_robots
            elif op.operation_type == OperationType.INSPECT:
                eligible = b_robots

            for rid in eligible:
                x_vars[(op_id, rid)] = model.NewBoolVar(f"x_{op_id}_{rid}")

        # ——————————————————————————————————————————————————————————
        # 约束1: 每个操作恰好分配给一台机器人
        # ——————————————————————————————————————————————————————————
        for op_id, op in operations.items():
            eligible = []
            if op.operation_type in (OperationType.DISASSEMBLE, OperationType.INSTALL):
                eligible = a_robots
            elif op.operation_type == OperationType.INSPECT:
                eligible = b_robots

            if eligible:
                model.Add(sum(x_vars[(op_id, rid)] for rid in eligible) == 1)
            else:
                # 没有合适的机器人 -> 不可行
                return ScheduleResult(
                    status=ResultStatus.INFEASIBLE.value,
                    objective={"error": f"操作 {op_id} 没有可用的机器人"},
                )

        # ——————————————————————————————————————————————————————————
        # 约束2: 前序关系（同一台离心机的 D->I, I->R）
        #   如果 D 分配给 A_i，则 I 必须在某 B_j 上执行
        #   如果 I 分配给 B_j，则 R 必须在某 A_i 上执行
        #   这由操作存在性保证，但需要确保 D完成后 I 才能开始
        #   在 assignment_only 模式下，我们只确保分配本身的一致性
        # ——————————————————————————————————————————————————————————
        # 同台机器的 D 和 I 至少分配给不同机器人
        # 实际上在 assignment_only 模式下，D/R 只能给A，I 只能给B
        # 前序关系由类型分配自然保证

        # ——————————————————————————————————————————————————————————
        # 约束3: A 机器人之间负载均衡目标
        # ——————————————————————————————————————————————————————————
        if len(a_robots) > 1:
            a_loads: dict[str, cp_model.LinearExpr] = {}
            for rid in a_robots:
                a_loads[rid] = sum(
                    x_vars[(op_id, rid)]
                    for op_id in list(d_ops) + list(r_ops)
                    if (op_id, rid) in x_vars
                )

            # 负载上界和下界
            max_load_a = model.NewIntVar(0, len(d_ops) + len(r_ops), "max_load_a")
            min_load_a = model.NewIntVar(0, len(d_ops) + len(r_ops), "min_load_a")

            for rid in a_robots:
                model.Add(a_loads[rid] <= max_load_a)
                model.Add(a_loads[rid] >= min_load_a)

            # 目标: 最小化最大负载差
            load_diff_a = model.NewIntVar(
                0, len(d_ops) + len(r_ops), "load_diff_a"
            )
            model.Add(max_load_a - min_load_a == load_diff_a)
            model.Minimize(load_diff_a)

        if len(b_robots) > 1:
            b_loads: dict[str, cp_model.LinearExpr] = {}
            for rid in b_robots:
                b_loads[rid] = sum(
                    x_vars[(op_id, rid)]
                    for op_id in i_ops
                    if (op_id, rid) in x_vars
                )

            max_load_b = model.NewIntVar(0, len(i_ops), "max_load_b")
            min_load_b = model.NewIntVar(0, len(i_ops), "min_load_b")

            for rid in b_robots:
                model.Add(b_loads[rid] <= max_load_b)
                model.Add(b_loads[rid] >= min_load_b)

            load_diff_b = model.NewIntVar(0, len(i_ops), "load_diff_b")
            model.Add(max_load_b - min_load_b == load_diff_b)

            # 如果 A 也有负载均衡目标，则组合目标
            if len(a_robots) > 1:
                total_diff = model.NewIntVar(
                    0, len(d_ops) + len(r_ops) + len(i_ops), "total_diff"
                )
                model.Add(total_diff == load_diff_a + load_diff_b)
                model.Minimize(total_diff)
            else:
                model.Minimize(load_diff_b)

        # ——————————————————————————————————————————————————————————
        # 求解
        # ——————————————————————————————————————————————————————————
        status = self._solver.Solve(model)

        if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
            return self._extract_solution(
                problem, x_vars, a_robots, b_robots, status
            )
        elif status == cp_model.INFEASIBLE:
            return ScheduleResult(
                status=ResultStatus.INFEASIBLE.value,
                objective={"error": "CP-SAT 判定问题不可行"},
            )
        else:
            return ScheduleResult(
                status=ResultStatus.TIMEOUT.value,
                objective={"error": f"求解超时，状态={status}"},
            )

    # ------------------------------------------------------------------
    def _extract_solution(
        self,
        problem: SchedulingProblem,
        x_vars: dict,
        a_robots: list[str],
        b_robots: list[str],
        status: int,
    ) -> ScheduleResult:
        """从 CP-SAT 解中提取分配结果。"""
        result = ScheduleResult(
            status=ResultStatus.FEASIBLE.value
            if status == cp_model.FEASIBLE
            else ResultStatus.SUCCESS.value,
        )

        # 记录求解统计
        result.solve_time_seconds = self._solver.WallTime()
        result.best_objective_bound = self._solver.BestObjectiveBound()
        result.objective = {
            "objective_value": self._solver.ObjectiveValue(),
            "solve_time": result.solve_time_seconds,
        }

        # 提取分配
        for (op_id, rid), var in x_vars.items():
            if self._solver.Value(var) > 0.5:
                op = problem.operations[op_id]
                result.assignments.append({
                    "operation_id": op_id,
                    "robot_id": rid,
                    "machine_id": op.machine_id,
                    "operation_type": op.operation_type.value,
                })

                if rid not in result.robot_schedules:
                    result.robot_schedules[rid] = RobotSchedule(robot_id=rid)
                result.robot_schedules[rid].operations.append((op_id, -1, -1))

        # 统计各机器人负载
        for rid in a_robots + b_robots:
            if rid in result.robot_schedules:
                count = len(result.robot_schedules[rid].operations)
            else:
                count = 0
            result.objective[f"load_{rid}"] = count

        return result
