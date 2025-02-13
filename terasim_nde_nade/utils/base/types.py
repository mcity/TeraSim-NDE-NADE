from enum import Enum
from typing import Union, TypeVar

class AgentType(Enum):
    """Type of agents in the simulation."""
    VEHICLE = "vehicle"
    VRU = "vru"

class VRUType(Enum):
    """Type of Vulnerable Road Users."""
    PEDESTRIAN = "pedestrian"
    CYCLIST = "cyclist"

class CommandType(Enum):
    """CommandType types for agent control."""
    DEFAULT = "default"
    LEFT = "left"
    RIGHT = "right"
    TRAJECTORY = "trajectory"
    ACCELERATION = "acceleration"
    CUSTOM = "custom"

# Type variables for type hinting
Agent = TypeVar('Agent', 'Vehicle', 'VRU') 