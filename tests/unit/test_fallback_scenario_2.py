from src.domain.enums import RobotType
from src.domain.models import Cell, Footprint, RobotSpec
from src.map.fixed_map import FixedMap
from src.solver.fallback import manual_assign_scenario_2


def test_scenario_2_partition_pipeline():
    _, machines, operations = FixedMap().build()
    foot = Footprint.default_2x4()
    robots = {
        "A_1": RobotSpec("A_1", RobotType.A, Cell(1, 28), foot),
        "A_2": RobotSpec("A_2", RobotType.A, Cell(12, 28), foot),
        "B_1": RobotSpec("B_1", RobotType.B, Cell(24, 28), foot),
    }
    result = manual_assign_scenario_2(machines, operations, robots)
    assert len(result.assignments) == 144
    assert len(result.robot_schedules["A_1"].operations) == 48
    assert len(result.robot_schedules["A_2"].operations) == 48
    assert len(result.robot_schedules["B_1"].operations) == 48
    a1_first = result.robot_schedules["A_1"].operations[0][0]
    a2_first = result.robot_schedules["A_2"].operations[0][0]
    assert a1_first == "M_y3_x2_D"
    assert a2_first == "M_y3_x5_D"
    assert result.robot_schedules["A_1"].operations[24][0] == "M_y23_x2_R"
    assert result.robot_schedules["A_2"].operations[24][0] == "M_y23_x5_R"
