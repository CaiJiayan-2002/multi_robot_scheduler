"""状态机单元测试 v4.0

测试 MachineStateMachine 状态转移逻辑。
基于实际 API: can_start_operation, lock_machine, unlock_machine, complete_operation,
             all_completed, completed_count, summary
"""

from __future__ import annotations

import pytest

from src.domain.enums import MachineState, OperationType, RobotType
from src.domain.models import Cell, Machine
from src.simulation.state_machine import MachineStateMachine


# ── Fixtures ─────────────────────────────────────────────────────────────────


@pytest.fixture
def sample_machines():
    """创建 5 台测试离心机（初始状态: PENDING_DISASSEMBLY=7）。"""
    return {
        "M_y3_x2": Machine(
            machine_id="M_y3_x2", cells=(Cell(2, 3), Cell(3, 3)), row=3,
            state=MachineState.PENDING_DISASSEMBLY,
        ),
        "M_y3_x5": Machine(
            machine_id="M_y3_x5", cells=(Cell(5, 3), Cell(6, 3)), row=3,
            state=MachineState.PENDING_DISASSEMBLY,
        ),
        "M_y7_x2": Machine(
            machine_id="M_y7_x2", cells=(Cell(2, 7), Cell(3, 7)), row=7,
            state=MachineState.PENDING_DISASSEMBLY,
        ),
        "M_y7_x5": Machine(
            machine_id="M_y7_x5", cells=(Cell(5, 7), Cell(6, 7)), row=7,
            state=MachineState.PENDING_DISASSEMBLY,
        ),
        "M_y11_x2": Machine(
            machine_id="M_y11_x2", cells=(Cell(2, 11), Cell(3, 11)), row=11,
            state=MachineState.PENDING_DISASSEMBLY,
        ),
    }


@pytest.fixture
def sm(sample_machines):
    """创建状态机实例。"""
    return MachineStateMachine(sample_machines)


# ── 枚举值测试 ──────────────────────────────────────────────────────────────


class TestMachineStateEnum:
    """直接测试 MachineState 枚举定义。"""

    def test_initial_state_is_7(self):
        assert MachineState.PENDING_DISASSEMBLY.value == 7

    def test_inspection_state_is_8(self):
        assert MachineState.PENDING_INSPECTION.value == 8

    def test_installation_state_is_9(self):
        assert MachineState.PENDING_INSTALLATION.value == 9

    def test_completed_state_is_10(self):
        assert MachineState.COMPLETED.value == 10

    def test_state_order(self):
        states = [
            MachineState.PENDING_DISASSEMBLY,
            MachineState.PENDING_INSPECTION,
            MachineState.PENDING_INSTALLATION,
            MachineState.COMPLETED,
        ]
        for i in range(1, len(states)):
            assert states[i].value > states[i - 1].value


# ── 状态转移规则 ───────────────────────────────────────────────────────────


class TestMachineStateTransitions:
    """can_start_operation 规则测试。"""

    def test_cannot_skip_states(self, sm):
        """状态 7 时不能执行 INSTALL（跳过 INSPECT）。"""
        ok, reason = sm.can_start_operation("M_y3_x2", OperationType.INSTALL)
        assert not ok, f"Should not allow INSTALL in state 7: {reason}"

    def test_disassemble_allowed_in_state_7(self, sm):
        """状态 7 时可以执行 DISASSEMBLE。"""
        ok, _ = sm.can_start_operation("M_y3_x2", OperationType.DISASSEMBLE)
        assert ok, "Should allow DISASSEMBLE when state is PENDING_DISASSEMBLY"

    def test_inspect_allowed_in_state_8(self, sm, sample_machines):
        """状态 8 时可以执行 INSPECT。"""
        sample_machines["M_y3_x2"].state = MachineState.PENDING_INSPECTION
        ok, _ = sm.can_start_operation("M_y3_x2", OperationType.INSPECT)
        assert ok, "Should allow INSPECT when state is PENDING_INSPECTION"

    def test_install_allowed_in_state_9(self, sm, sample_machines):
        """状态 9 时可以执行 INSTALL。"""
        sample_machines["M_y3_x2"].state = MachineState.PENDING_INSTALLATION
        ok, _ = sm.can_start_operation("M_y3_x2", OperationType.INSTALL)
        assert ok, "Should allow INSTALL when state is PENDING_INSTALLATION"

    def test_cannot_double_disassemble(self, sm, sample_machines):
        """状态 8 时不能再执行 DISASSEMBLE。"""
        sample_machines["M_y3_x2"].state = MachineState.PENDING_INSPECTION
        ok, reason = sm.can_start_operation("M_y3_x2", OperationType.DISASSEMBLE)
        assert not ok, f"Should not allow DISASSEMBLE in state 8: {reason}"

    def test_cannot_inspect_completed(self, sm, sample_machines):
        """状态 10 时不能执行任何操作。"""
        sample_machines["M_y3_x2"].state = MachineState.COMPLETED
        ok, _ = sm.can_start_operation("M_y3_x2", OperationType.INSPECT)
        assert not ok, "Should not allow INSPECT when COMPLETED"


# ── 锁定机制 ──────────────────────────────────────────────────────────────


class TestMachineLocking:
    """lock_machine / unlock_machine 测试。"""

    def test_lock_acquires(self, sm, sample_machines):
        """lock_machine 成功锁定未锁定的机器。"""
        result = sm.lock_machine("M_y3_x2", "A_1")
        assert result, "First lock should succeed"
        assert sample_machines["M_y3_x2"].locked_by == "A_1"

    def test_other_robot_cannot_lock(self, sm):
        """锁定后其他机器人不能 lock 同一台。"""
        sm.lock_machine("M_y3_x2", "A_1")
        result = sm.lock_machine("M_y3_x2", "B_1")
        assert not result, "Second lock by different robot should fail"

    def test_lock_idempotent_for_same_robot(self, sm):
        """同一机器人可以重新锁定（幂等）。"""
        sm.lock_machine("M_y3_x2", "A_1")
        result = sm.lock_machine("M_y3_x2", "A_1")
        assert result, "Same robot should be able to re-lock"

    def test_unlock_releases(self, sm):
        """unlock 后可以重新 lock。"""
        sm.lock_machine("M_y3_x2", "A_1")
        sm.unlock_machine("M_y3_x2")
        result = sm.lock_machine("M_y3_x2", "B_1")
        assert result, "Should be able to lock after unlock"

    def test_unlock_always_succeeds(self, sm):
        """unlock_machine 总是成功（即使未被锁定）。"""
        # 不应抛出异常
        sm.unlock_machine("M_y3_x2")

    def test_lock_nonexistent_machine(self, sm):
        """锁定不存在的机器返回 False。"""
        result = sm.lock_machine("NONEXISTENT", "A_1")
        assert not result, "Should return False for nonexistent machine"


# ── 状态推进 ──────────────────────────────────────────────────────────────


class TestStateAdvancement:
    """complete_operation 测试。"""

    def test_complete_disassemble_moves_to_8(self, sm):
        """完成 DISASSEMBLE 后状态 = PENDING_INSPECTION (8)。"""
        sm.complete_operation("M_y3_x2", OperationType.DISASSEMBLE)
        assert sm.machines["M_y3_x2"].state == MachineState.PENDING_INSPECTION

    def test_complete_inspect_moves_to_9(self, sm, sample_machines):
        """完成 INSPECT 后状态 = PENDING_INSTALLATION (9)。"""
        sample_machines["M_y3_x2"].state = MachineState.PENDING_INSPECTION
        sm.complete_operation("M_y3_x2", OperationType.INSPECT)
        assert sm.machines["M_y3_x2"].state == MachineState.PENDING_INSTALLATION

    def test_complete_install_moves_to_10(self, sm, sample_machines):
        """完成 INSTALL 后状态 = COMPLETED (10)。"""
        sample_machines["M_y3_x2"].state = MachineState.PENDING_INSTALLATION
        sm.complete_operation("M_y3_x2", OperationType.INSTALL)
        assert sm.machines["M_y3_x2"].state == MachineState.COMPLETED

    def test_complete_nonexistent_raises(self, sm):
        """对不存在的机器调用 complete_operation 应抛出 ValueError。"""
        with pytest.raises(ValueError):
            sm.complete_operation("NONEXISTENT", OperationType.DISASSEMBLE)

    def test_full_chain_d_i_r(self, sm):
        """完整操作链: D -> I -> R 推进到状态 10。"""
        sm.complete_operation("M_y3_x2", OperationType.DISASSEMBLE)
        assert sm.machines["M_y3_x2"].state == MachineState.PENDING_INSPECTION

        sm.complete_operation("M_y3_x2", OperationType.INSPECT)
        assert sm.machines["M_y3_x2"].state == MachineState.PENDING_INSTALLATION

        sm.complete_operation("M_y3_x2", OperationType.INSTALL)
        assert sm.machines["M_y3_x2"].state == MachineState.COMPLETED


# ── all_completed / completed_count ──────────────────────────────────────


class TestAllCompleted:
    """all_completed / completed_count 测试。"""

    def test_all_completed_false_initially(self, sm):
        """初始状态下 all_completed = False。"""
        assert not sm.all_completed()
        assert sm.completed_count() == 0

    def test_all_completed_true_when_done(self, sm, sample_machines):
        """所有机器 COMPLETED 时 all_completed = True。"""
        for m in sample_machines.values():
            m.state = MachineState.COMPLETED
        assert sm.all_completed()
        assert sm.completed_count() == 5

    def test_partial_complete(self, sm, sample_machines):
        """部分完成时 all_completed = False。"""
        sample_machines["M_y3_x2"].state = MachineState.COMPLETED
        assert not sm.all_completed()
        assert sm.completed_count() == 1

    def test_48_machines_all_completed(self):
        """48 台机器全部完成时 all_completed = True。"""
        machines = {}
        for i in range(48):
            mid = f"M_test_{i}"
            machines[mid] = Machine(
                machine_id=mid,
                cells=(Cell(1, 1), Cell(2, 1)),
                row=1,
                state=MachineState.COMPLETED,
            )
        sm48 = MachineStateMachine(machines)
        assert sm48.all_completed()
        assert sm48.completed_count() == 48


# ── get_machines_ready_for ────────────────────────────────────────────────


class TestGetMachinesReady:
    """get_machines_ready_for 测试。"""

    def test_ready_for_disassemble_initially(self, sm):
        """初始时所有 5 台机器都对 DISASSEMBLE 就绪。"""
        ready = sm.get_machines_ready_for(OperationType.DISASSEMBLE)
        assert len(ready) == 5

    def test_ready_when_locked(self, sm):
        """锁定后该机器不再就绪。"""
        sm.lock_machine("M_y3_x2", "A_1")
        ready = sm.get_machines_ready_for(OperationType.DISASSEMBLE)
        assert "M_y3_x2" not in ready
        assert len(ready) == 4

    def test_ready_after_state_change(self, sm):
        """状态改变后对新操作就绪。"""
        sm.complete_operation("M_y3_x2", OperationType.DISASSEMBLE)
        # 现在对 INSPECT 就绪
        ready = sm.get_machines_ready_for(OperationType.INSPECT)
        assert "M_y3_x2" in ready

        # 但对 DISASSEMBLE 不再就绪
        ready_d = sm.get_machines_ready_for(OperationType.DISASSEMBLE)
        assert "M_y3_x2" not in ready_d


# ── get_machines_in_state ────────────────────────────────────────────────


class TestGetMachinesInState:
    """get_machines_in_state 测试。"""

    def test_all_in_state_7_initially(self, sm):
        """初始时全部在 PENDING_DISASSEMBLY。"""
        ms = sm.get_machines_in_state(MachineState.PENDING_DISASSEMBLY)
        assert len(ms) == 5

    def test_after_completion(self, sm):
        """完成后在 COMPLETED 状态。"""
        sm.complete_operation("M_y3_x2", OperationType.DISASSEMBLE)
        sm.complete_operation("M_y3_x2", OperationType.INSPECT)
        sm.complete_operation("M_y3_x2", OperationType.INSTALL)

        completed = sm.get_machines_in_state(MachineState.COMPLETED)
        assert "M_y3_x2" in completed
        assert len(completed) == 1


# ── summary ───────────────────────────────────────────────────────────────


class TestSummary:
    """summary 测试。"""

    def test_summary_has_all_states(self, sm):
        """summary 应包含所有状态和 total。"""
        s = sm.summary()
        assert "PENDING_DISASSEMBLY" in s
        assert "PENDING_INSPECTION" in s
        assert "PENDING_INSTALLATION" in s
        assert "COMPLETED" in s
        assert "total" in s
        assert s["total"] == 5

    def test_summary_counts_initial(self, sm):
        """初始时 PENDING_DISASSEMBLY = 5, 其余 = 0。"""
        s = sm.summary()
        assert s["PENDING_DISASSEMBLY"] == 5
        assert s["PENDING_INSPECTION"] == 0
        assert s["PENDING_INSTALLATION"] == 0
        assert s["COMPLETED"] == 0
