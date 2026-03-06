from enum import Enum, auto


class CombatState(Enum):
    ATTACK = auto()
    DEFENCE = auto()
    STANDBY = auto()
