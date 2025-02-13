from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Dict, List, Optional


@dataclass
class AgentInfo:
    """Base class for agent information."""

    id: str
    position: List[float]
    velocity: float
    heading: float

    def __getitem__(self, item):
        return self.__dict__[item]
