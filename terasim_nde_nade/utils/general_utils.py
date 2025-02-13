import numpy as np
from enum import Enum
from pydantic import BaseModel
from typing import List, Dict, Any, Optional, Callable
import math
from terasim.overlay import traci

class Command(Enum):
    DEFAULT = "default"
    LEFT = "left"
    RIGHT = "right"
    TRAJECTORY = "trajectory"
    ACCELERATION = "acceleration"
    CUSTOM = "custom"

class NDECommand(BaseModel):
    """
    Represents a command for an agent in a Non-Deterministic Environment (NDE).
    if the command is "default", the agent will follow the SUMO controlled model, other elements will be ignored
    if the command is "left" or "right", the agent will change lane to the left or right, other elements will be ignored
    if the command is "trajectory", the agent will follow the future trajectory, which will be predicted according to the current acceleration, other elements will be ignored
    if the command is "acceleration", the agent will decelerate to stop using the acceleration element
    """
    command_type: Command = Command.DEFAULT
    acceleration: float = 0.0
    future_trajectory: List[List] = [[]]  # shape: (n, 5) for [x, y, heading, velocity, time]
    prob: float = 1.0
    duration: float = None
    info: Dict[str, Any] = {}
    custom_control_command: Dict[str, Any] = None
    custom_execute_control_command: Callable = None
    keep_route_mode: int = 1

    @validator("duration", pre=True, always=True)
    def set_duration(cls, v):
        return v if v is not None else traci.simulation.getDeltaT()

    class Config:
        slots = True
        extra = "forbid"

def angle_difference(angle1: float, angle2: float) -> float:
    """Compute the absolute difference between two angles in degrees."""
    # Compute the difference between the two angles and reduce it to the range [-180, 180]
    diff = (angle1 - angle2 + 180) % 360 - 180
    return abs(diff)

def get_sumo_angle(np_angle: float) -> float:
    """Convert numpy angle to SUMO angle format."""
    sumo_angle = (90 - np_angle) % 360
    return sumo_angle

def predict_future_distance_velocity_vectorized(
    velocity: float,
    acceleration: float,
    duration_array: np.ndarray,
    max_velocity: float,
) -> np.ndarray:
    """Predict the future distance of the agent using vectorized operations for improved performance.

    Args:
        velocity (float): The initial velocity of the agent.
        acceleration (float): The acceleration of the agent.
        duration_array (np.ndarray): The array of time points at which to calculate distance.
        max_velocity (float): The maximum velocity of the agent.

    Returns:
        np.ndarray: The array of future distances at each time point in duration_array.
    """
    # Calculate velocity at each time point, ensuring it does not exceed max_velocity
    velocity_array = np.clip(velocity + acceleration * duration_array, 0, max_velocity)

    # Calculate the average velocities between consecutive time points
    average_velocities = 0.5 * (velocity_array[1:] + velocity_array[:-1])

    # Calculate the time differences between consecutive time points
    time_differences = duration_array[1:] - duration_array[:-1]

    # Calculate distance increments using the average velocities and time differences
    distance_increments = average_velocities * time_differences

    # Calculate the cumulative distance at each time point
    cumulative_distances = np.cumsum(distance_increments)
    cumulative_distances = np.insert(cumulative_distances, 0, 0)  # Include starting point (distance=0)

    return cumulative_distances, velocity_array 