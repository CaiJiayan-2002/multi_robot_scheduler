"""
场景1 (1A1B) 完整运行 — 输出全部10项评价指标 + 可视化

输出:
1. 机器人运行轨迹动画 (HTML)
2. 机器人任务甘特图 (PNG + HTML)
3. 作业总时间
4. 单机器人与系统总路径长度
5. 各类等待时间
6. 碰撞、约束违规次数
7. 各机器人利用率
8. 负载均衡指标
9. 算法计算时间
10. 结果汇总 JSON
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.domain.enums import RobotType
from src.domain.models import Cell, Footprint, RobotSpec
from src.map.fixed_map import FixedMap
from src.map.pose_graph import PoseGraph
from src.map.service_poses import ServicePoseCalculator
from src.planning.static_astar import StaticAStar
from src.simulation.engine import SimulationEngine
from src.solver.fallback import manual_assign_scenario_1
from src.evaluation.metrics import MetricsCalculator
from src.evaluation.plots import GanttChart, TrajectoryPlot


def main():
    timing: dict[str, float] = {}
    t0 = time.perf_counter()

    # ==================================================================
    # 1. 地图预处理
    # ==================================================================
    print("=" * 70)
    print("  Scenario 1: 1A + 1B — Full Evaluation")
    print("=" * 70)

    t_map = time.perf_counter()
    fm = FixedMap()
    terrain, machines, operations = fm.build()
    footprint = Footprint.default_2x4()
    timing["map_preprocess"] = time.perf_counter() - t_map
    print(f"\n[1] Map: {terrain.shape}, {len(machines)} machines, "
          f"{len(operations)} operations  ({timing['map_preprocess']:.3f}s)")

    # ==================================================================
    # 2. 姿态图 + 旅行矩阵
    # ==================================================================
    t_pose = time.perf_counter()
    pose_graph = PoseGraph(terrain, footprint)
    pose_graph.build()
    static_astar = StaticAStar(pose_graph)

    robot_specs = {
        "A_1": RobotSpec("A_1", RobotType.A, Cell(2, 25), footprint),
        "B_1": RobotSpec("B_1", RobotType.B, Cell(5, 25), footprint),
    }

    # 预计算旅行时间
    service_anchors = {
        mid: ServicePoseCalculator.compute_service_anchor(m, footprint)
        for mid, m in machines.items()
    }
    sources = {rid: rs.start_anchor for rid, rs in robot_specs.items()}
    travel_matrix = static_astar.precompute_travel_matrix(sources, service_anchors)
    timing["map_preprocess"] += time.perf_counter() - t_pose
    print(f"[2] PoseGraph: {pose_graph.node_count()} nodes, "
          f"{pose_graph.edge_count()} edges")

    # ==================================================================
    # 3. 任务分配 (Fallback)
    # ==================================================================
    t_solver = time.perf_counter()
    schedule = manual_assign_scenario_1(
        {mid: m for mid, m in machines.items()},
        operations,
        robot_specs,
    )
    timing["solver"] = time.perf_counter() - t_solver
    a_ops = len(schedule.robot_schedules["A_1"].operations)
    b_ops = len(schedule.robot_schedules["B_1"].operations)
    print(f"[3] Assignment: A_1={a_ops} ops, B_1={b_ops} ops  "
          f"({timing['solver']:.3f}s)")

    # ==================================================================
    # 4. 仿真运行
    # ==================================================================
    t_sim = time.perf_counter()
    engine = SimulationEngine()
    engine.setup(
        terrain=terrain,
        machines={mid: m for mid, m in machines.items()},
        operations=operations,
        robots=robot_specs,
        schedule=schedule,
    )
    engine.run(max_steps=30000)
    timing["simulation"] = time.perf_counter() - t_sim
    timing["total_wall"] = time.perf_counter() - t0

    event_log = engine.event_log
    makespan = engine.current_time
    print(f"[4] Simulation: {makespan} time steps, "
          f"{len(event_log)} events  ({timing['simulation']:.2f}s)")

    # ==================================================================
    # 5. 计算指标
    # ==================================================================
    print(f"\n{'=' * 70}")
    print(f"  EVALUATION METRICS")
    print(f"{'=' * 70}")

    scenario_metrics = MetricsCalculator.compute(
        event_log=event_log,
        robots_info=engine.robots,
        state_machine=engine.state_machine,
        make_span=makespan,
        timing=timing,
    )

    # ----- 指标 3: 作业总时间 -----
    print(f"\n[3] MAKESPAN (total time): {scenario_metrics.makespan} steps")

    # ----- 指标 4: 路径长度 -----
    print(f"\n[4] PATH LENGTH:")
    for rid, plen in scenario_metrics.path_by_robot.items():
        rm = scenario_metrics.robot_metrics[rid]
        print(f"    {rid} ({rm.robot_type}): {plen} steps "
              f"(move={rm.move_time}, work={rm.work_time})")
    print(f"    TOTAL: {scenario_metrics.total_path_length} steps")

    # ----- 指标 5: 等待时间 -----
    print(f"\n[5] WAIT TIMES:")
    print(f"    Precedence wait : {scenario_metrics.total_wait_precedence}")
    print(f"    Conflict wait   : {scenario_metrics.total_wait_conflict}")
    print(f"    Idle wait       : {scenario_metrics.total_wait_idle}")
    print(f"    TOTAL wait      : {scenario_metrics.total_wait}")
    for rid, w in scenario_metrics.wait_by_robot.items():
        print(f"    {rid}: precedence={w['precedence']}, "
              f"conflict={w['conflict']}, idle={w['idle']}, "
              f"total={w['total']}")

    # ----- 指标 6: 碰撞与违规 -----
    print(f"\n[6] COLLISIONS & VIOLATIONS:")
    print(f"    Collisions      : {scenario_metrics.collision_count}")
    print(f"    Constraint violations: {scenario_metrics.constraint_violation_count}")
    print(f"    Precedence violations: {scenario_metrics.precedence_violation_count}")

    # ----- 指标 7: 利用率 -----
    print(f"\n[7] UTILIZATION:")
    for rid, u in scenario_metrics.utilization_by_robot.items():
        print(f"    {rid}: service={u['service_utilization_pct']:.1f}%, "
              f"movement={u['movement_utilization_pct']:.1f}%, "
              f"wait={u['wait_ratio_pct']:.1f}%")

    # ----- 指标 8: 负载均衡 -----
    print(f"\n[8] LOAD BALANCE:")
    print(f"    A-type CV (variation): {scenario_metrics.load_cv_a}")
    print(f"    B-type CV (variation): {scenario_metrics.load_cv_b}")
    print(f"    A-type gap (max-min):  {scenario_metrics.load_gap_a}")
    print(f"    B-type gap (max-min):  {scenario_metrics.load_gap_b}")

    # ----- 指标 9: 算法时间 -----
    print(f"\n[9] ALGORITHM TIME:")
    for phase, elapsed in timing.items():
        print(f"    {phase}: {elapsed:.3f}s")
    print(f"    TOTAL WALL TIME: {timing['total_wall']:.2f}s")

    # ----- 指标 10: 规划质量 -----
    print(f"\n[10] PLANNING QUALITY:")
    print(f"    Replans: {scenario_metrics.number_of_replans}")
    print(f"    Solver status: {schedule.status}")
    print(f"    Fallback used: {schedule.fallback_used}")
    machine_summary = engine.state_machine.summary()
    print(f"    Machines completed: {machine_summary.get('COMPLETED', 0)}/48")
    for state, count in machine_summary.items():
        print(f"      {state}: {count}")

    # ==================================================================
    # 6. 生成可视化
    # ==================================================================
    output_dir = Path(__file__).resolve().parent.parent / "outputs" / "scenario_1"
    output_dir.mkdir(parents=True, exist_ok=True)

    # 甘特图
    print(f"\n{'=' * 70}")
    print(f"  VISUALIZATIONS")
    print(f"{'=' * 70}")

    gantt_data = GanttChart.build_gantt_data(event_log)
    GanttChart.save_gantt_png(
        gantt_data,
        str(output_dir / "gantt.png"),
        title=f"Scenario 1 (1A1B) Gantt Chart — Makespan={makespan}",
    )

    # 轨迹
    trajectories = TrajectoryPlot.build_trajectory_data(event_log)
    TrajectoryPlot.save_trajectory_json(
        trajectories, str(output_dir / "trajectories.json")
    )
    TrajectoryPlot.save_trajectory_png(
        trajectories, terrain,
        {mid: m for mid, m in machines.items()},
        str(output_dir / "trajectories.png"),
        title=f"Scenario 1 (1A1B) — {makespan} steps, {len(event_log)} events",
    )

    # 完整指标 JSON
    metrics_json = {
        "scenario": "1A1B",
        "makespan": scenario_metrics.makespan,
        "path_length": {
            "total": scenario_metrics.total_path_length,
            "by_robot": scenario_metrics.path_by_robot,
            "by_type": scenario_metrics.path_by_type,
        },
        "wait_times": {
            "total_precedence": scenario_metrics.total_wait_precedence,
            "total_conflict": scenario_metrics.total_wait_conflict,
            "total_idle": scenario_metrics.total_wait_idle,
            "total": scenario_metrics.total_wait,
            "by_robot": scenario_metrics.wait_by_robot,
        },
        "collisions_violations": {
            "collisions": scenario_metrics.collision_count,
            "constraint_violations": scenario_metrics.constraint_violation_count,
            "precedence_violations": scenario_metrics.precedence_violation_count,
        },
        "utilization": scenario_metrics.utilization_by_robot,
        "load_balance": {
            "cv_a": scenario_metrics.load_cv_a,
            "cv_b": scenario_metrics.load_cv_b,
            "gap_a": scenario_metrics.load_gap_a,
            "gap_b": scenario_metrics.load_gap_b,
        },
        "timing": {
            phase: round(t, 4) for phase, t in timing.items()
        },
        "planning_quality": {
            "replans": scenario_metrics.number_of_replans,
            "solver_status": schedule.status,
            "fallback_used": schedule.fallback_used,
        },
        "machine_completion": machine_summary,
    }

    metrics_path = output_dir / "metrics.json"
    with open(metrics_path, "w") as f:
        json.dump(metrics_json, f, indent=2, ensure_ascii=False)
    print(f"\n[DATA] Full metrics saved: {metrics_path}")

    # 事件日志
    log_path = output_dir / "event_log.jsonl"
    with open(log_path, "w") as f:
        for e in event_log:
            f.write(json.dumps(e, ensure_ascii=False) + "\n")
    print(f"[DATA] Event log saved: {log_path}  ({len(event_log)} events)")

    print(f"\n{'=' * 70}")
    print(f"  ALL OUTPUTS: {output_dir}/")
    print(f"    gantt.png          — Gantt chart (PNG)")
    print(f"    trajectories.png   — Trajectory map (PNG)")
    print(f"    trajectories.json  — Trajectory data (JSON)")
    print(f"    metrics.json       — All 10 metrics (JSON)")
    print(f"    event_log.jsonl    — Simulation events ({len(event_log)} events)")
    print(f"{'=' * 70}")


if __name__ == "__main__":
    main()
