"""领域枚举定义 v4.0"""

from enum import Enum, auto


class TerrainCode(Enum):
    OBSTACLE = 0
    INTERNAL_ROAD = 1
    TRUNK_ROAD = 2


class RobotType(Enum):
    A = "A"
    B = "B"


class MachineState(Enum):
    PENDING_DISASSEMBLY = 7
    PENDING_INSPECTION = 8
    PENDING_INSTALLATION = 9
    COMPLETED = 10


class OperationType(Enum):
    DISASSEMBLE = "DISASSEMBLE"
    INSPECT = "INSPECT"
    INSTALL = "INSTALL"


class Action(Enum):
    UP = (0, -1)
    DOWN = (0, 1)
    LEFT = (-1, 0)
    RIGHT = (1, 0)
    WAIT = (0, 0)


class RobotStatus(Enum):
    IDLE = auto()
    MOVING = auto()
    ALIGNING = auto()
    WORKING = auto()
    WAITING_PRECEDENCE = auto()
    WAITING_CONFLICT = auto()
    FINISHED = auto()


class ResultStatus(Enum):
    SUCCESS = "success"
    FEASIBLE = "feasible"
    INFEASIBLE = "infeasible"
    TIMEOUT = "timeout"
    INVALID_INPUT = "invalid_input"
    INTERNAL_ERROR = "internal_error"
