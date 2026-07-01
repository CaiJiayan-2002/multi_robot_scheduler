"""评估指标计算模块 v4.0

从仿真事件日志计算全部评价指标。
对照 v4.0 文档 Section 14 定义。

事件日志格式示例:
  move:    "A_1: -> (2,24) t=1 UP"
  work_start: "A_1: start DISASSEMBLE on M_y3_x2, duration=6"
  work_complete: "A_1: completed DISASSEMBLE on M_y3_x2, machine state=PENDING_INSPECTION"
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import re


@dataclass
class RobotMetrics:
    """单台机器人的指标"""
    robot_id: str = ""
    robot_type: str = ""
    path_length: int = 0       # 移动步数（不含WAIT/START）
    move_time: int = 0         # 移动耗时
    work_time: int = 0         # 作业耗时
    wait_precedence: int = 0   # 等待前置条件
    wait_conflict: int = 0     # 等待预约冲突
    wait_idle: int = 0         # 空闲等待
    wait_total: int = 0        # 总等待
    operations_completed: int = 0
    finish_time: int = 0
    service_utilization: float = 0.0
    movement_utilization: float = 0.0
    wait_ratio: float = 0.0


@dataclass
class ScenarioMetrics:
    """场景级指标 (v4.0 Section 14)"""
    scenario_name: str = ""
    makespan: int = 0
    total_path_length: int = 0
    path_by_robot: dict[str, int] = field(default_factory=dict)
    path_by_type: dict[str, int] = field(default_factory=dict)
    total_wait_precedence: int = 0
    total_wait_conflict: int = 0
    total_wait_idle: int = 0
    total_wait: int = 0
    wait_by_robot: dict[str, dict] = field(default_factory=dict)
    collision_count: int = 0
    constraint_violation_count: int = 0
    precedence_violation_count: int = 0
    utilization_by_robot: dict[str, dict] = field(default_factory=dict)
    load_cv_a: float = 0.0
    load_cv_b: float = 0.0
    load_gap_a: float = 0.0
    load_gap_b: float = 0.0
    map_preprocess_time: float = 0.0
    solver_time: float = 0.0
    initial_path_planning_time: float = 0.0
    simulation_time: float = 0.0
    total_wall_time: float = 0.0
    number_of_replans: int = 0
    solver_status: str = ""
    fallback_used: bool = False
    robot_metrics: dict[str, RobotMetrics] = field(default_factory=dict)


class MetricsCalculator:
    """从仿真事件日志计算所有评价指标。"""

    # 匹配 robot ID: A_1, B_1, A_2, etc.
    RID_RE = re.compile(r'^([AB]_\d+):')

    @staticmethod
    def compute(
        event_log: list[dict],
        robots_info: dict,
        state_machine,
        make_span: int,
        timing: dict | None = None,
    ) -> ScenarioMetrics:
        timing = timing or {}
        metrics = ScenarioMetrics()
        metrics.makespan = make_span

        # 收集所有 robot IDs
        all_rids: set[str] = set()
        for e in event_log:
            m = MetricsCalculator.RID_RE.match(e.get("message", ""))
            if m:
                all_rids.add(m.group(1))
        # fallback: from robots_info
        if not all_rids and robots_info:
            all_rids = set(robots_info.keys())

        for rid in sorted(all_rids):
            rm = RobotMetrics(
                robot_id=rid,
                robot_type="A" if rid.startswith("A_") else "B",
            )
            MetricsCalculator._compute_robot(rm, rid, event_log, make_span)
            metrics.robot_metrics[rid] = rm

        # --- 路径 ---
        for rid, rm in metrics.robot_metrics.items():
            metrics.total_path_length += rm.path_length
            metrics.path_by_robot[rid] = rm.path_length
            t = rm.robot_type
            metrics.path_by_type[t] = metrics.path_by_type.get(t, 0) + rm.path_length

        # --- 等待 ---
        for rid, rm in metrics.robot_metrics.items():
            metrics.total_wait_precedence += rm.wait_precedence
            metrics.total_wait_conflict += rm.wait_conflict
            metrics.total_wait_idle += rm.wait_idle
            metrics.total_wait += rm.wait_total
            metrics.wait_by_robot[rid] = {
                "precedence": rm.wait_precedence,
                "conflict": rm.wait_conflict,
                "idle": rm.wait_idle,
                "total": rm.wait_total,
            }

        # --- 碰撞 ---
        metrics.collision_count = sum(
            1 for e in event_log if e.get("type") == "collision"
        )
        metrics.constraint_violation_count = sum(
            1 for e in event_log if "violation" in e.get("type", "")
        )

        # --- 利用率 ---
        for rid, rm in metrics.robot_metrics.items():
            ms = make_span if make_span > 0 else 1
            rm.service_utilization = round(rm.work_time / ms * 100, 2)
            rm.movement_utilization = round(rm.move_time / ms * 100, 2)
            rm.wait_ratio = round(rm.wait_total / ms * 100, 2)
            metrics.utilization_by_robot[rid] = {
                "service_utilization_pct": rm.service_utilization,
                "movement_utilization_pct": rm.movement_utilization,
                "wait_ratio_pct": rm.wait_ratio,
            }

        # --- 负载均衡 ---
        MetricsCalculator._load_balance(metrics)

        # --- 时间 ---
        metrics.map_preprocess_time = timing.get("map_preprocess", 0)
        metrics.solver_time = timing.get("solver", 0)
        metrics.simulation_time = timing.get("simulation", 0)
        metrics.total_wall_time = sum(timing.values())

        # --- 规划质量 ---
        metrics.number_of_replans = sum(
            1 for e in event_log if e.get("type") in ("replan", "retry")
        )

        return metrics

    @staticmethod
    def _compute_robot(
        rm: RobotMetrics, rid: str, event_log: list[dict], makespan: int,
    ) -> None:
        """从事件日志统计单台机器人的各项耗时"""
        last_finish = 0

        for e in event_log:
            msg = e.get("message", "")
            if not msg.startswith(f"{rid}:"):
                continue
            etype = e.get("type", "")

            if etype == "move":
                # "A_1: -> (2,24) t=1 UP"
                action_part = msg.split()[-1] if msg.split() else ""
                if action_part not in ("WAIT", "START"):
                    rm.path_length += 1
                rm.move_time += 1

            elif etype == "work_tick":
                rm.work_time += 1

            elif etype == "work_complete":
                rm.operations_completed += 1
                t = e.get("t", 0)
                if t > last_finish:
                    last_finish = t

            elif etype == "wait_precedence" or etype == "precedence_cleared":
                rm.wait_precedence += 1

            elif etype == "retry":
                rm.wait_conflict += 1

        rm.finish_time = last_finish

        # 总等待 = makespan - 活跃时间（移动+作业）
        active = rm.move_time + rm.work_time
        rm.wait_total = max(0, makespan - active)
        # idle = 剩余未分类等待
        rm.wait_idle = max(
            0, rm.wait_total - rm.wait_precedence - rm.wait_conflict
        )

    @staticmethod
    def _load_balance(metrics: ScenarioMetrics) -> None:
        a_loads = [
            rm.work_time + rm.move_time
            for rm in metrics.robot_metrics.values()
            if rm.robot_type == "A"
        ]
        b_loads = [
            rm.work_time + rm.move_time
            for rm in metrics.robot_metrics.values()
            if rm.robot_type == "B"
        ]
        for label, loads, cv_attr, gap_attr in [
            ("A", a_loads, "load_cv_a", "load_gap_a"),
            ("B", b_loads, "load_cv_b", "load_gap_b"),
        ]:
            if len(loads) >= 2 and sum(loads) > 0:
                mean_l = sum(loads) / len(loads)
                var = sum((l - mean_l) ** 2 for l in loads) / len(loads)
                cv = math.sqrt(var) / mean_l
                gap = max(loads) - min(loads)
                setattr(metrics, cv_attr, round(cv, 4))
                setattr(metrics, gap_attr, round(gap, 2))
            elif len(loads) == 1:
                setattr(metrics, cv_attr, 0.0)
                setattr(metrics, gap_attr, 0.0)
