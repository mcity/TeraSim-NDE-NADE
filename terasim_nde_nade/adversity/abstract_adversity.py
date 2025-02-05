import abc
import time
from abc import abstractmethod
from dataclasses import dataclass
from typing import Any, List, Optional, Type, Dict
import addict


class AbstractAdversity(abc.ABC):
    def __new__(cls, *args: Any, **kwargs: Any):
        instance: AbstractAdversity = super().__new__(cls)
        instance._adversity_output = []
        return instance

    def __init__(
        self,
        location,
        ego_type,
        probability,
        predicted_collision_type,
    ):
        self._location = location
        self._ego_type = ego_type
        self._probability = probability
        self._predicted_collision_type = predicted_collision_type
        self._negligence_command_dict = addict.Dict()
    
    @abstractmethod
    def trigger(self, obs_dict: Dict) -> bool:
        pass

    @abstractmethod
    def derive_command(self, obs_dict: Dict) -> addict.Dict:
        pass