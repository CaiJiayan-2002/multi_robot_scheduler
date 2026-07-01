"""场景 1 端到端集成测试 v4.0

验证完整 pipeline: 地图生成 -> fallback手动分配 -> 预约表 -> 时空A*规划 -> 仿真引擎 -> 碰撞=0

场景 1: 1A1B (A_1 做所有拆卸+安装, B_1 做所有检测)
"""

from __future__ import annotations

import time

import pytest

from src.domain.enums import MachineState, OperationType, RobotType
from src.domain.models import Cell, Footprint, Machine, RobotSpec, SchedulingProblem
from src.map.fixed_map import FixedMap
from src.map.pose_graph import PoseGraph
from src.planning.static_astar import StaticAStar
from src.solver.fallback import manual_assign_scenario_1


# ── 模块可用性检测 ──────────────────────────────────────────────────────────

try:
    from src.simulation.engine import SimulationEngine
    _HAS_ENGINE = True
except ImportError:
    _HAS_ENGINE = False


# ── Fixtures ─────────────────────────────────────────────────────────────────


@pytest.fixture(scope="module")
def fmap():
    fm = FixedMap()
    fm.build()
    return fm


@pytest.fixture(scope="module")
def terrain(fmap):
    return fmap.terrain


@pytest.fixture(scope="module")
def machines(fmap):
    return fmap.machines


@pytest.fixture(scope="module")
def operations(fmap):
    return fmap.operations


@pytest.fixture(scope="module")
def footprint():
    return Footprint.default_2x4()


@pytest.fixture(scope="module")
def robot_specs():
    return {
        "A_1": RobotSpec(
            robot_id="A_1", robot_type=RobotType.A,
            start_anchor=Cell(1, 24),  # 主干道左端
        ),
        "B_1": RobotSpec(
            robot_id="B_1", robot_type=RobotType.B,
            start_anchor=Cell(24, 24),  # 主干道右端
        ),
    }


# ── 1. W1 模块验证 ────────────────────────────────────────────────────────


class TestScenario1W1Modules:
    """验证 W1 模块在场景中正常工作。"""

    def test_map_generated(self, terrain, machines, operations):
        assert terrain.shape == (29, 25)
        assert len(machines) == 48
        assert len(operations) == 144

    def test_pose_graph_built(self, terrain, footprint):
        pg = PoseGraph(terrain, footprint, trunk_y_threshold=24)
        pg.build()
        assert pg.node_count() > 0
        assert pg.edge_count() > 0

    def test_static_astar_reaches_all_machines(self, terrain, footprint, operations, robot_specs):
        pg = PoseGraph(terrain, footprint, trunk_y_threshold=24)
        pg.build()
        astar = StaticAStar(pg)

        for rid, spec in robot_specs.items():
            for op_id, op in operations.items():
                if op.eligible_robot_type == spec.robot_type:
                    result = astar.plan(spec.start_anchor, op.service_anchor)
                    assert result is not None, \
                        f"{rid} cannot reach {op_id} at ({op.service_anchor.x},{op.service_anchor.y})"


# ── 2. Fallback 分配验证 ─────────────────────────────────────────────────


class TestFallbackAssignment:
    """验证 fallback 手动分配逻辑。"""

    def test_manual_assign_scenario_1(self, machines, operations, robot_specs):
        """场景 1: A_1=96 ops (48D+48R), B_1=48 ops (48I)."""
        result = manual_assign_scenario_1(machines, operations, robot_specs)

        assert result.status == "feasible"
        assert result.fallback_used is True

        # A_1 应有 96 个操作
        a_sched = result.robot_schedules.get("A_1")
        assert a_sched is not None, "A_1 should have a schedule"
        assert len(a_sched.operations) == 96, \
            f"A_1 should have 96 ops, got {len(a_sched.operations)}"

        # B_1 应有 48 个操作
        b_sched = result.robot_schedules.get("B_1")
        assert b_sched is not None, "B_1 should have a schedule"
        assert len(b_sched.operations) == 48, \
            f"B_1 should have 48 ops, got {len(b_sched.operations)}"

    def test_manual_assign_operation_types(self, machines, operations, robot_specs):
        """分配的操作类型正确。"""
        result = manual_assign_scenario_1(machines, operations, robot_specs)

        a_ops = result.robot_schedules["A_1"].operations
        for op_id, _, _ in a_ops:
            op = operations[op_id]
            assert op.operation_type != OperationType.INSPECT, \
                f"A_1 has INSPECT operation {op_id}"
            assert op.eligible_robot_type == RobotType.A, \
                f"{op_id} not eligible for A"

        b_ops = result.robot_schedules["B_1"].operations
        for op_id, _, _ in b_ops:
            op = operations[op_id]
            assert op.operation_type == OperationType.INSPECT, \
                f"B_1 has non-INSPECT operation {op_id}"
            assert op.eligible_robot_type == RobotType.B, \
                f"{op_id} not eligible for B"

    def test_manual_assign_precedence_order(self, machines, operations, robot_specs):
        """验证 D 操作在 R 操作之前（按机器排序）。"""
        result = manual_assign_scenario_1(machines, operations, robot_specs)
        a_ops = result.robot_schedules["A_1"].operations

        # 所有 D 操作在 R 操作之前
        d_indices = [i for i, (op_id, _, _) in enumerate(a_ops) if op_id.endswith("_D")]
        r_indices = [i for i, (op_id, _, _) in enumerate(a_ops) if op_id.endswith("_R")]

        max_d = max(d_indices) if d_indices else -1
        min_r = min(r_indices) if r_indices else float("inf")

        assert max_d < min_r, \
            f"All D ops should precede R ops: max D index={max_d}, min R index={min_r}"

    def test_manual_assign_assignments_list(self, machines, operations, robot_specs):
        """assignments 列表应包含 144 个条目。"""
        result = manual_assign_scenario_1(machines, operations, robot_specs)
        assert len(result.assignments) == 144, \
            f"Expected 144 assignments, got {len(result.assignments)}"


# ── 3. 仿真引擎验证（核心 pipeline）──────────────────────────────────────


class TestScenario1Simulation:
    """使用 Agent1 的 SimulationEngine 运行完整仿真。"""

    def test_simulation_engine_import(self):
        """验证 SimulationEngine 可导入。"""
        if not _HAS_ENGINE:
            pytest.skip("SimulationEngine module not available")
        assert SimulationEngine is not None

    def test_setup_no_error(self, terrain, machines, operations, robot_specs):
        """setup 不抛出异常。"""
        if not _HAS_ENGINE:
            pytest.skip("SimulationEngine not available")

        schedule = manual_assign_scenario_1(machines, operations, robot_specs)
        engine = SimulationEngine()
        engine.setup(terrain, machines, operations, robot_specs, schedule)
        assert engine.current_time == 0
        assert len(engine.robots) == 2

    def test_step_produces_state(self, terrain, machines, operations, robot_specs):
        """单步仿真返回有效状态。"""
        if not _HAS_ENGINE:
            pytest.skip("SimulationEngine not available")

        schedule = manual_assign_scenario_1(machines, operations, robot_specs)
        engine = SimulationEngine()
        engine.setup(terrain, machines, operations, robot_specs, schedule)

        state = engine.step()
        assert "time" in state
        assert "robots" in state
        assert "machines" in state

    def test_run_to_completion(self, terrain, machines, operations, robot_specs):
        """运行到 48 台机器全部完成。"""
        if not _HAS_ENGINE:
            pytest.skip("SimulationEngine not available")

        schedule = manual_assign_scenario_1(machines, operations, robot_specs)
        engine = SimulationEngine()
        engine.setup(terrain, machines, operations, robot_specs, schedule)

        t0 = time.time()
        event_log = engine.run(max_steps=10000)
        elapsed = time.time() - t0

        # 验证仿真结束
        assert engine.is_finished(), "Simulation should be finished"
        assert engine.state_machine.all_completed(), \
            "All machines should be COMPLETED"

        # 输出统计
        final_state = engine.get_state()
        print(f"\n  Simulation finished at t={final_state['time']}")
        print(f"  Elapsed: {elapsed:.1f}s")
        print(f"  Machine summary: {final_state['machines']}")
        print(f"  Events: {len(event_log)}")

        # 每台机器应处于 COMPLETED
        assert final_state["machines"]["COMPLETED"] == 48, \
            f"Expected 48 completed, got {final_state['machines']}"

    def test_no_collisions_in_log(self, terrain, machines, operations, robot_specs):
        """事件日志中无碰撞记录。"""
        if not _HAS_ENGINE:
            pytest.skip("SimulationEngine not available")

        schedule = manual_assign_scenario_1(machines, operations, robot_specs)
        engine = SimulationEngine()
        engine.setup(terrain, machines, operations, robot_specs, schedule)
        engine.run(max_steps=10000)

        # 检查是否有碰撞相关事件
        collision_events = engine.get_events_by_type("collision")
        assert len(collision_events) == 0, \
            f"Found {len(collision_events)} collision events"

    def test_all_operations_completed(self, terrain, machines, operations, robot_specs):
        """验证所有 144 个操作都被执行。"""
        if not _HAS_ENGINE:
            pytest.skip("SimulationEngine not available")

        schedule = manual_assign_scenario_1(machines, operations, robot_specs)
        engine = SimulationEngine()
        engine.setup(terrain, machines, operations, robot_specs, schedule)
        engine.run(max_steps=10000)

        # 汇总所有机器人完成的操作
        total_completed = sum(len(r.completed_ops) for r in engine.robots.values())
        assert total_completed == 144, \
            f"Expected 144 completed operations, got {total_completed}"


# ── 4. 简化版手动画 pipeline 验证（不依赖 engine）──────────────────────────


class TestManualPipelineWithoutEngine:
    """不依赖 SimulationEngine 的简化 pipeline 验证。

    使用 fallback 分配 + 静态 A* 规划移动作路径 + 简单状态推进 + 碰撞检测。
    """

    def test_full_static_pipeline(self, terrain, footprint, machines, operations, robot_specs):
        """完整简化 pipeline: 分配 -> A*规划 -> 状态推进 -> 碰撞检测。

        使用静态 A* 规划所有机器人的顺序任务（无预约表冲突避免），
        验证在顺序执行下无碰撞。
        """
        pg = PoseGraph(terrain, footprint, trunk_y_threshold=24)
        pg.build()
        astar = StaticAStar(pg)

        assignment = manual_assign_scenario_1(machines, operations, robot_specs)

        # 模拟顺序执行：A_1 执行全部操作，然后 B_1 执行
        all_timed_poses: list[tuple[str, Cell, int]] = []  # (robot_id, cell, t)
        global_t = 0

        # A_1 先执行所有操作
        a_ops = assignment.robot_schedules["A_1"].operations
        current_pos = robot_specs["A_1"].start_anchor
        for op_id, _, _ in a_ops:
            op = operations[op_id]
            target = op.service_anchor

            # 规划路径
            result = astar.plan(current_pos, target)
            assert result is not None, f"A_1 cannot reach {op_id}"
            path, cost = result

            # 记录移动路径
            for cell in path:
                all_timed_poses.append(("A_1", cell, global_t))
                global_t += 1

            # 服务等待
            for dt in range(op.duration):
                all_timed_poses.append(("A_1", target, global_t))
                global_t += 1

            current_pos = target

        # B_1 执行所有操作
        b_ops = assignment.robot_schedules["B_1"].operations
        current_pos = robot_specs["B_1"].start_anchor
        for op_id, _, _ in b_ops:
            op = operations[op_id]
            target = op.service_anchor

            result = astar.plan(current_pos, target)
            assert result is not None, f"B_1 cannot reach {op_id}"
            path, cost = result

            for cell in path:
                all_timed_poses.append(("B_1", cell, global_t))
                global_t += 1

            for dt in range(op.duration):
                all_timed_poses.append(("B_1", target, global_t))
                global_t += 1

            current_pos = target

        # 碰撞检测
        max_t = max(p[2] for p in all_timed_poses)
        collisions = 0
        for t in range(max_t + 1):
            cell_owners: dict[tuple[int, int], set[str]] = {}
            for rid, cell, pose_t in all_timed_poses:
                if pose_t != t:
                    continue
                body = footprint.cells_at(cell)
                for c in body:
                    key = (c.x, c.y)
                    if key not in cell_owners:
                        cell_owners[key] = set()
                    cell_owners[key].add(rid)

            for pos, robots_here in cell_owners.items():
                if len(robots_here) > 1:
                    collisions += 1

        print(f"\n  Manual pipeline: {len(all_timed_poses)} poses over {max_t} time steps")
        print(f"  Collisions: {collisions}")
        assert collisions == 0, f"Found {collisions} collisions in sequential execution"


# ── 5. 场景统计 ──────────────────────────────────────────────────────────


class TestScenario1Statistics:
    """场景 1 统计报告。"""

    def test_scenario_1_summary(self, terrain, footprint, machines, operations, robot_specs):
        """输出场景 1 统计。"""
        assignment = manual_assign_scenario_1(machines, operations, robot_specs)

        a_ops = assignment.robot_schedules["A_1"].operations
        b_ops = assignment.robot_schedules["B_1"].operations

        a_d_ops = [oid for oid, _, _ in a_ops if oid.endswith("_D")]
        a_r_ops = [oid for oid, _, _ in a_ops if oid.endswith("_R")]

        print("\n" + "=" * 60)
        print("  SCENARIO 1 SUMMARY (1A1B)")
        print("=" * 60)
        print(f"  Machines: {len(machines)}")
        print(f"  Operations: {len(operations)}")
        print(f"  A_1: {len(a_ops)} ops ({len(a_d_ops)} D + {len(a_r_ops)} R)")
        print(f"  B_1: {len(b_ops)} ops ({len(b_ops)} I)")
        print(f"  A_1 work time: {len(a_d_ops)*6 + len(a_r_ops)*6} steps (D+R)")
        print(f"  B_1 work time: {len(b_ops)*10} steps (I)")
        print(f"  Assignments: {len(assignment.assignments)}")
        print(f"  Fallback used: {assignment.fallback_used}")
        print("=" * 60)
