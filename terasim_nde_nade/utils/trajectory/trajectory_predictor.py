"""Trajectory utilities for TeraSim NDE/NADE."""
from typing import Any, Dict, List, Tuple

import addict
import numpy as np
from loguru import logger
from terasim.overlay import profile, traci

from ..agents.vehicle import (VehicleInfoForPredict,
                              get_lanechange_longitudinal_speed,
                              get_vehicle_info)
from ..base.types import CommandType


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
    cumulative_distances = np.insert(
        cumulative_distances, 0, 0
    )  # Include starting point (distance=0)

    return cumulative_distances, velocity_array


def get_vehicle_future_lane_id_from_edge(
    edge_id: str, upcoming_lane_id_list: List[str]
) -> str:
    """Get the future lane ID for a vehicle given its edge ID."""
    return next(
        (lane_id for lane_id in upcoming_lane_id_list if edge_id in lane_id), None
    )


def get_future_lane_id_index(
    veh_id: str,
    veh_edge_id: str,
    upcoming_lane_id_list: List[str],
    original_lane_index: int,
    lateral_offset: int,
) -> Tuple[str, int]:
    """Get the future lane ID and index for a vehicle."""
    if traci.edge.getLaneNumber(veh_edge_id) == 1:
        veh_lane_index = 0
        veh_lane_id = veh_edge_id + "_0"
        return veh_lane_id, veh_lane_index
    else:
        max_lane_index = traci.edge.getLaneNumber(veh_edge_id) - 1
        predicted_vehicle_lane_id = get_vehicle_future_lane_id_from_edge(
            veh_edge_id, upcoming_lane_id_list
        )
        if predicted_vehicle_lane_id is not None:
            predicted_veh_lane_index = int(predicted_vehicle_lane_id.split("_")[-1])
        else:
            predicted_veh_lane_index = original_lane_index

        veh_lane_index = min(
            max_lane_index, max(0, predicted_veh_lane_index + lateral_offset)
        )
        veh_lane_id = veh_edge_id + f"_{veh_lane_index}"
        return veh_lane_id, veh_lane_index


def get_future_position_on_route(
    traci,
    veh_id: str,
    veh_edge_id: str,
    veh_lane_position: float,
    veh_lane_index: int,
    veh_lane_id: str,
    veh_route_id_list: List[str],
    veh_route_length_list: List[float],
    future_distance: float,
    future_lateral_offset: int,
    upcoming_lane_id_list: List[str],
) -> Tuple[Tuple[float, float], float]:
    """Predict the future position of a vehicle on its route."""
    veh_lane_position += future_distance
    current_lane_length = traci.lane.getLength(veh_lane_id)

    if veh_edge_id not in veh_route_id_list:
        found_match = False
        # Try matching with base edge id
        edge_base = veh_edge_id.split("_")[0]
        min_dist = float("inf")
        modified_index = 0

        for i, route_id in enumerate(veh_route_id_list):
            route_base = route_id.split("_")[0]
            if edge_base == route_base:
                edge_pos = traci.simulation.convert2D(veh_edge_id, 0, 0)
                route_pos = traci.simulation.convert2D(route_id, 0, 0)
                dist = (
                    (edge_pos[0] - route_pos[0]) ** 2
                    + (edge_pos[1] - route_pos[1]) ** 2
                ) ** 0.5
                if dist < min_dist:
                    min_dist = dist
                    modified_index = i

        veh_route_id_list[modified_index] = veh_edge_id

        if min_dist > 10:
            logger.warning(
                f"Edge {veh_edge_id} not found in route list {veh_route_id_list}"
            )

    current_route_index = veh_route_id_list.index(veh_edge_id)

    # Calculate the corresponding edge and lane position
    while (
        veh_lane_position > current_lane_length
        and current_route_index < len(veh_route_id_list) - 1
    ):
        current_route_index += 1
        veh_edge_id = veh_route_id_list[current_route_index]
        veh_lane_position -= current_lane_length
        current_lane_length = veh_route_length_list[current_route_index]

    # Calculate the new lane index
    veh_lane_id, veh_lane_index = get_future_lane_id_index(
        veh_id,
        veh_edge_id,
        upcoming_lane_id_list,
        veh_lane_index,
        future_lateral_offset,
    )

    veh_lane_position = min(
        veh_lane_position,
        current_lane_length,
        traci.lane.getLength(veh_lane_id) - 0.1,
    )
    future_position = traci.simulation.convert2D(
        veh_edge_id, veh_lane_position, veh_lane_index
    )
    future_heading = traci.lane.getAngle(veh_lane_id, veh_lane_position)
    return future_position, future_heading


@profile
def predict_future_trajectory(
    veh_id: str,
    obs_dict: Dict[str, Any],
    control_command: Any,
    sumo_net,
    time_horizon_step: int = 4,
    time_resolution: float = 0.5,
    interpolate_resolution: float = 0.1,
    current_time: float = None,
    veh_info: VehicleInfoForPredict = None,
) -> Tuple[np.ndarray, Dict[str, Any]]:
    """Predict the future trajectory of the vehicle.
    All position and heading in this function stays the same definition with sumo,
    which is (x, y) and angle in degree (north is 0, east is 90, south is 180, west is 270)
    """
    info = addict.Dict()
    current_time = (
        current_time if current_time is not None else traci.simulation.getTime()
    )
    veh_info = (
        veh_info
        if veh_info is not None
        else get_vehicle_info(veh_id, obs_dict, sumo_net)
    )

    # Include the original position
    duration_array = np.array(
        [
            time_horizon_id * time_resolution
            for time_horizon_id in range(time_horizon_step + 1)
        ]
    )

    acceleration = (
        control_command.acceleration
        if control_command.command_type == CommandType.ACCELERATION
        else veh_info["acceleration"]
    )
    max_velocity = traci.vehicle.getAllowedSpeed(veh_id)

    lane_width = traci.lane.getWidth(veh_info["lane_id"])
    lateral_offset = 0
    if control_command.command_type == CommandType.LEFT:
        lateral_offset = 1
        veh_info.velocity = get_lanechange_longitudinal_speed(
            veh_id,
            veh_info.velocity,
            lane_width,
        )
    elif control_command.command_type == CommandType.RIGHT:
        lateral_offset = -1
        veh_info.velocity = get_lanechange_longitudinal_speed(
            veh_id,
            veh_info.velocity,
            lane_width,
        )

    (
        future_distance_array,
        future_velocity_array,
    ) = predict_future_distance_velocity_vectorized(
        veh_info["velocity"], acceleration, duration_array, max_velocity
    )

    trajectory_array = np.array(
        [
            veh_info.position[0],
            veh_info.position[1],
            veh_info.heading,
            future_velocity_array[0],
            0,
        ]
    )

    lanechange_finish_trajectory_point = None
    if (
        control_command.command_type == CommandType.LEFT
        or control_command.command_type == CommandType.RIGHT
    ):
        lanechange_finish_timestep = np.argmin(
            np.abs(duration_array - control_command.duration)
        )
        (
            lanechange_finish_position,
            lanechange_finish_final_heading,
        ) = get_future_position_on_route(
            traci,
            veh_id,
            veh_info["edge_id"],
            veh_info["lane_position"],
            veh_info["lane_index"],
            veh_info["lane_id"],
            veh_info["route_id_list"],
            veh_info["route_length_list"],
            future_distance_array[lanechange_finish_timestep],
            lateral_offset,
            veh_info["upcoming_lane_id_list"],
        )
        lanechange_finish_trajectory_point = np.array(
            [
                lanechange_finish_position[0],
                lanechange_finish_position[1],
                lanechange_finish_final_heading,
                future_velocity_array[lanechange_finish_timestep],
                duration_array[lanechange_finish_timestep],
            ]
        )
        trajectory_array = np.vstack(
            (trajectory_array, lanechange_finish_trajectory_point)
        )

    for duration, distance, velocity in zip(
        duration_array[1:], future_distance_array[1:], future_velocity_array[1:]
    ):
        if (
            lanechange_finish_trajectory_point is not None
            and duration <= lanechange_finish_trajectory_point[-1]
        ):
            continue
        future_position, future_heading = get_future_position_on_route(
            traci,
            veh_id,
            veh_info["edge_id"],
            veh_info["lane_position"],
            veh_info["lane_index"],
            veh_info["lane_id"],
            veh_info["route_id_list"],
            veh_info["route_length_list"],
            distance,
            lateral_offset,
            veh_info["upcoming_lane_id_list"],
        )
        trajectory_array = np.vstack(
            (
                trajectory_array,
                np.array(
                    [
                        future_position[0],
                        future_position[1],
                        future_heading,
                        velocity,
                        duration,
                    ]
                ),
            )
        )

    future_trajectory_array = trajectory_array
    future_trajectory_array[:, -1] += current_time
    return future_trajectory_array, info
