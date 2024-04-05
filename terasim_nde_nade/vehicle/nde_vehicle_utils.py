import numpy as np
import math
from terasim.overlay import traci
import attr
import sumolib
from terasim.utils import (
    sumo_coordinate_to_center_coordinate,
    sumo_heading_to_orientation,
)
import json
from enum import Enum
from collections import namedtuple
from scipy.interpolate import interp1d

# Define the TrajectoryPoint named tuple
TrajectoryPoint = namedtuple("TrajectoryPoint", ["timestep", "position", "heading"])
from terasim_nde_nade.vehicle.nde_vehicle_utils_cython import *

from typing import List, Tuple, Dict, Any, Optional, Callable
from pydantic import BaseModel, validator
from enum import Enum


intersection_cutin_prob = (
    1.0648925125333899e-04
    * 0.10
    * 0.61
    * 1.5
    * 2.9
    * 0.5
    * 0.73
    * 1.38
    * 1.05
    * 0.86
    * 0.87
)
intersection_neglect_conflict_lead_prob = (
    5.5069126130818786e-05
    * 2.67
    * 0.61
    * 1.39
    * 0.89
    * 1.57
    * 0.7
    * 1.48
    * 0.82
    * 0.8
    * 2.35
    * 0.6
    * 0.8
    * 0.5
)
intersection_rearend_prob = (
    1.0149373787617288e-06
    * 2.78
    * 0.44
    * 0.29
    * 0.62
    * 2
    * 2
    * 1.56
    * 0.67
    * 2
    * 0.5
    * 1.5
    * 1.5
    * 1.5
    * 0.7
)
intersection_tfl_prob = (
    8.797353696327892e-04
    * 0.37
    * 0.13
    * 2.67
    * 0.75
    * 1.17
    * 0.5
    * 0.5
    * 2
    * 2
    * 0.5
    * 0.5
    * 10
    * 5
    * 1.23
    * 1.2
)
intersection_headon_prob = (
    1.0113094565177164e-06
    * 4.71
    * 0.1
    * 2.57
    * 0.25
    * 0.5
    * 2
    * 2
    * 1.42
    * 1.76
    * 2
    * 2
    * 1.74
    * 0.8
    * 1.32
    * 0.8
)

roundabout_fail_to_yield_prob = 1e-7 * 10 * 2 * 2 * 2 * 2 * 0.9 * 1.1 * 0.81
roundabout_cutin_prob = (
    4.570171651138106e-05 * 0.35 * 0.6 * 0.5 * 2 * 0.77 * 2 * 0.63 * 1.34 * 1.2 * 0.9
)
roundabout_neglect_conflict_lead_prob = (
    2.3721857594970477e-05 * 0.1 * 0.33 * 0.2 * 2 * 2 * 2 * 2 * 2 * 0.5 * 0.63 * 1.19
)
roundabout_rearend_prob = (
    4.884970702773788e-07 * 0.2 * 5 * 0.5 * 2 * 2 * 0.5 * 0.84 * 0.5 * 1.4 * 0.8
)

highway_cutin_prob = (
    9.887380418491011e-05
    * 0.232
    * 1.65
    * 1.23
    * 0.36
    * 2
    * 1.27
    * 0.5
    * 1.34
    * 2
    * 0.5
    * 0.75
    * 1.1
    * 1.1
)
highway_rearend_prob = (
    1.0327637301820217e-04
    * 3.64
    * 1.21
    * 1.63
    * 2
    * 2
    * 2
    * 0.5
    * 0.5
    * 1.4
    * 0.83
    * 0.81
    * 0.92
    * 1.15
)


class Command(Enum):
    DEFAULT = "default"
    LEFT = "left"
    RIGHT = "right"
    TRAJECTORY = "trajectory"
    ACCELERATION = "acceleration"
    CUSTOM = "custom"


class NDECommand(BaseModel):
    """
    Represents a command for a vehicle in a Non-Deterministic Environment (NDE).
    if the command is "default", the vehicle will follow the SUMO controlled model, other elements will be ignored
    if the command is "left" or "right", the vehicle will change lane to the left or right, other elements will be ignored
    if the command is "trajectory", the vehicle will follow the future trajectory, which will be predicted according to the current acceleration, other elements will be ignored
    if the command is "acceleration", the vehicle will decelerate to stop using the acceleration element
    """

    command_type: Command = Command.DEFAULT
    acceleration: float = 0.0
    future_trajectory: List[Tuple[float, float]] = []
    prob: float = 1.0
    duration: float = None
    info: Dict[str, Any] = {}
    custom_control_command: Dict[str, Any] = None
    custom_execute_control_command: Callable = None

    @validator("duration", pre=True, always=True)
    def set_duration(cls, v):
        return v if v is not None else traci.simulation.getDeltaT()


def get_next_lane_edge(net, lane_id):
    origin_lane = net.getLane(lane_id)
    outgoing_lanes = [conn.getToLane() for conn in origin_lane.getOutgoing()]
    outgoing_edges = [lane.getEdge() for lane in outgoing_lanes]
    return outgoing_lanes[0].getID(), outgoing_edges[0].getID()


tls_controlled_lane_set = None
from itertools import chain


def cache_tls_controlled_lane_set():
    global tls_controlled_lane_set
    for tls_id in traci.trafficlight.getIDList():
        controlled_links = traci.trafficlight.getControlledLinks(tls_id)
        lane_set = {
            lane
            for link in chain.from_iterable(controlled_links)
            for lane in [link[0], link[-1]]
        }
        tls_controlled_lane_set.update(lane_set)


def get_distance_to_next_tls(veh_id: str):
    next_tls_info = traci.vehicle.getNextTLS(veh_id)
    if next_tls_info:
        tls_distance = next_tls_info[0][2]
        return tls_distance
    else:
        return float("inf")


def get_location(
    veh_id: str,
    lane_id: str = None,
    distance_to_tls_threshold: float = 25,
    highway_speed_threshold: float = 7.5,
    highlight_flag: bool = False,
):
    global tls_controlled_lane_set
    if tls_controlled_lane_set is None:
        tls_controlled_lane_set = set()
        cache_tls_controlled_lane_set()
    lane_id = lane_id if lane_id else traci.vehicle.getLaneID(veh_id)
    if traci.lane.getMaxSpeed(lane_id) > highway_speed_threshold:
        if highlight_flag:
            traci.vehicle.setColor(veh_id, (255, 0, 0, 255))  # red
        return "highway"
    elif (
        lane_id in tls_controlled_lane_set
        or get_distance_to_next_tls(veh_id) < distance_to_tls_threshold
    ):
        if highlight_flag:
            traci.vehicle.setColor(veh_id, (0, 255, 0, 255))  # green
        return "intersection"
    else:
        if highlight_flag:
            traci.vehicle.setColor(veh_id, (0, 0, 255, 255))  # blue
        return "roundabout"


def get_neighbour_lane(net, lane_id, direction="left"):
    if direction == "left":
        indexOffset = 1
    elif direction == "right":
        indexOffset = -1
    else:
        raise ValueError("direction must be either left or right")
    current_lane = net.getLane(lane_id)
    current_edge = current_lane.getEdge()
    all_lanes_on_edge = current_edge.getLanes()
    current_lane_index = all_lanes_on_edge.index(current_lane)
    neighbour_lane_index = current_lane_index + indexOffset
    neighbour_lane = all_lanes_on_edge[neighbour_lane_index]
    neighbour_lane_id = neighbour_lane.getID()
    return neighbour_lane_id


def get_sumo_angle(np_angle):
    sumo_angle = (90 - np_angle) % 360
    return sumo_angle


def get_lane_angle(lane_id, mode="start"):
    if mode == "start":
        relative_position = 0
    elif mode == "end":
        relative_position = traci.lane.getLength(lane_id) - 0.1
    else:
        raise ValueError("mode must be either start or end")
    lane_angle = traci.lane.getAngle(lane_id, relative_position)
    return lane_angle


def is_head_on(ego_obs, leader_info):
    ego_veh_lane_id = ego_obs["lane_id"] if ego_obs else None
    lead_veh_lane_id = traci.vehicle.getLaneID(leader_info[0]) if leader_info else None

    if ego_veh_lane_id is None or lead_veh_lane_id is None:
        return False

    ego_veh_lane_start_angle = get_lane_angle(ego_veh_lane_id, mode="start")
    lead_veh_lane_start_angle = get_lane_angle(lead_veh_lane_id, mode="start")
    ego_veh_lane_end_angle = get_lane_angle(ego_veh_lane_id, mode="end")
    lead_veh_lane_end_angle = get_lane_angle(lead_veh_lane_id, mode="end")

    if None in [
        ego_veh_lane_start_angle,
        lead_veh_lane_start_angle,
        ego_veh_lane_end_angle,
        lead_veh_lane_end_angle,
    ]:
        return False

    start_angle = abs(ego_veh_lane_start_angle - lead_veh_lane_start_angle)
    end_angle = abs(ego_veh_lane_end_angle - lead_veh_lane_end_angle)

    return (120 < start_angle < 240) and (120 < end_angle < 240)


def get_collision_type_and_prob(
    obs_dict: Dict[str, Any],
    negligence_command: NDECommand,
    location: Optional[str] = None,
) -> Tuple[float, str]:
    """
    Given current observation and the negligence mode, detect what type of collisions will be generated
    """
    if location is None:
        location = get_location(obs_dict["ego"]["veh_id"], obs_dict["ego"]["lane_id"])
    rear_end = negligence_command.info.get("is_car_following_flag", False)
    negligence_mode = negligence_command.info.get("negligence_mode", None)
    if "roundabout" in location:
        if negligence_mode == "LeftFoll" or negligence_mode == "RightFoll":
            return roundabout_cutin_prob, "roundabout_cutin"
        elif rear_end:
            return roundabout_rearend_prob, "roundabout_rearend"
        elif negligence_mode == "TrafficRule":
            return roundabout_fail_to_yield_prob, "roundabout_fail_to_yield"
        else:
            return (
                roundabout_neglect_conflict_lead_prob,
                "roundabout_neglect_conflict_lead",
            )
    elif "highway" in location:
        if negligence_mode == "LeftFoll" or negligence_mode == "RightFoll":
            return highway_cutin_prob, "highway_cutin"
        else:
            return highway_rearend_prob, "highway_rearend"
    elif "intersection" in location:
        if negligence_mode == "LeftFoll" or negligence_mode == "RightFoll":
            return intersection_cutin_prob, "intersection_cutin"
        elif rear_end:
            return intersection_rearend_prob, "intersection_rearend"
        elif is_head_on(
            obs_dict["ego"], negligence_command.info.get("leader_info", None)
        ):
            return intersection_headon_prob, "intersection_headon"
        elif negligence_mode == "TrafficRule":
            return intersection_tfl_prob, "intersection_tfl"
        else:
            return (
                intersection_neglect_conflict_lead_prob,
                "intersection_neglect_conflict_lead",
            )
    else:
        return 0, "no_collision"


from typing import List, Tuple


def is_car_following(follow_id: str, leader_id: str) -> bool:
    # check if the follow_id is car following the leader_id using the traci API and detect the future links
    current_edge_id = traci.vehicle.getLaneID(follow_id)
    leader_edge_id = traci.vehicle.getLaneID(leader_id)
    # the two vehicle are on the same link
    if current_edge_id == leader_edge_id:
        return True
    else:
        # the two vehicle are on different links, but the leader is on the future link of the follower
        follower_future_link_infos: List[Tuple] = traci.vehicle.getNextLinks(follow_id)
        if len(follower_future_link_infos) == 0:
            return False
        follower_future_lane_id: str = follower_future_link_infos[0][0]
        follower_future_junction_lane_id: str = follower_future_link_infos[0][
            4
        ]  # + "_0"
        if (
            leader_edge_id in follower_future_lane_id
            or leader_edge_id in follower_future_junction_lane_id
        ):
            return True

        leader_future_link_infos: List[Tuple] = traci.vehicle.getNextLinks(leader_id)
        if len(leader_future_link_infos) == 0:
            return False
        leader_future_lane_id: str = leader_future_link_infos[0][0]
        leader_junction_lane_id: str = leader_future_link_infos[0][4]
        if (
            len(
                set(
                    [
                        follower_future_lane_id,
                        follower_future_junction_lane_id,
                        leader_future_lane_id,
                        leader_junction_lane_id,
                    ]
                )
            )
            < 4  # the leader and follower has the same future link
        ):
            return True
        else:
            return False


def predict_future_distance_velocity_vectorized(
    velocity: float,
    acceleration: float,
    duration_array: np.ndarray,
    max_velocity: float,
) -> np.ndarray:
    """Predict the future distance of the vehicle using vectorized operations for improved performance.

    Args:
        velocity (float): The initial velocity of the vehicle.
        acceleration (float): The acceleration of the vehicle.
        duration_array (np.ndarray): The array of time points at which to calculate distance.
        max_velocity (float): The maximum velocity of the vehicle.

    Returns:
        np.ndarray: The array of future distances at each time point in duration_array.
    """
    # Calculate velocity at each time point, ensuring it does not exceed max_velocity.
    velocity_array = np.clip(velocity + acceleration * duration_array, 0, max_velocity)

    # Calculate the average velocities between consecutive time points.
    average_velocities = 0.5 * (velocity_array[1:] + velocity_array[:-1])

    # Calculate the time differences between consecutive time points.
    time_differences = duration_array[1:] - duration_array[:-1]

    # Calculate distance increments using the average velocities and time differences.
    distance_increments = average_velocities * time_differences

    # Calculate the cumulative distance at each time point.
    cumulative_distances = np.cumsum(distance_increments)
    cumulative_distances = np.insert(
        cumulative_distances, 0, 0
    )  # Include starting point (distance=0)

    return cumulative_distances, velocity_array


# @profile
def predict_future_trajectory(
    veh_id,
    obs_dict,
    control_command,
    sumo_net,
    time_horizon_step=4,
    time_resolution=0.5,
    interpolate_resolution=0.1,
    current_time=None,
    veh_info=None,
):
    """Predict the future trajectory of the vehicle.
    all position and heading in this function stays the same definition with sumo, which is (x, y) and angle in degree (north is 0, east is 90, south is 180, west is 270)

    Args:
        veh_id (str): the id of the vehicle
        time_horizon_step (int): the time horizon of the prediction
        time_resolution (float): the time resolution of the prediction

    Returns:
        future_trajectory_dict (dict): the future trajectory of the vehicle
    """
    current_time = (
        current_time if current_time is not None else traci.simulation.getTime()
    )
    veh_info = (
        veh_info
        if veh_info is not None
        else get_vehicle_info(veh_id, obs_dict, sumo_net)
    )
    # include the original position
    duration_array = np.array(
        [
            time_horizon_id * time_resolution
            for time_horizon_id in range(time_horizon_step + 1)
        ]
    )
    acceleration = (
        control_command.acceleration
        if control_command.command_type == Command.ACCELERATION
        else veh_info["acceleration"]
    )
    max_velocity = traci.vehicle.getAllowedSpeed(veh_id)
    future_distance_array, future_velocity_array = (
        predict_future_distance_velocity_vectorized(
            veh_info["velocity"], acceleration, duration_array, max_velocity
        )
    )
    lateral_offset = 0
    if control_command.command_type == Command.LEFT:
        lateral_offset = 1
    elif control_command.command_type == Command.RIGHT:
        lateral_offset = -1

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
        control_command.command_type == Command.LEFT
        or control_command.command_type == Command.RIGHT
    ):
        lanechange_finish_timestep = np.argmin(
            np.abs(duration_array - control_command.duration)
        )
        lanechange_finish_position, lanechange_finish_final_heading = (
            get_future_position_on_route(
                traci,
                veh_info["edge_id"],
                veh_info["lane_position"],
                veh_info["lane_index"],
                veh_info["lane_id"],
                veh_info["route_id_list"],
                veh_info["route_length_list"],
                future_distance_array[lanechange_finish_timestep],
                lateral_offset,
            )
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
            and duration <= lanechange_finish_trajectory_point[3]
        ):
            continue
        future_position, future_heading = get_future_position_on_route(
            traci,
            veh_info["edge_id"],
            veh_info["lane_position"],
            veh_info["lane_index"],
            veh_info["lane_id"],
            veh_info["route_id_list"],
            veh_info["route_length_list"],
            distance,
            lateral_offset,
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

    future_trajectory_array = interpolate_future_trajectory(
        trajectory_array, interpolate_resolution
    )

    future_trajectory_array[:, -1] += current_time
    return future_trajectory_array


def get_future_position_on_route(
    traci,
    veh_edge_id: str,
    veh_lane_position: float,
    veh_lane_index: int,
    veh_lane_id: str,
    veh_route_id_list: List[str],
    veh_route_length_list: List[float],
    future_distance: float,
    future_lateral_offset: int,
) -> Tuple[Tuple[float, float], float]:
    """
    Given the current vehicle edge id, lane position, current lane id, and the future distance / future lateral offset, predict the future position of the vehicle.
    """
    veh_lane_position += future_distance
    current_lane_length = traci.lane.getLength(veh_lane_id)
    current_route_index = veh_route_id_list.index(veh_edge_id)

    # calculate the corresponding edge and lane position
    while (
        veh_lane_position > current_lane_length
        and current_route_index < len(veh_route_id_list) - 1
    ):
        current_route_index += 1
        veh_edge_id = veh_route_id_list[current_route_index]
        veh_lane_position -= current_lane_length
        current_lane_length = veh_route_length_list[current_route_index]

    # calculate the new lane index
    max_lane_index = traci.edge.getLaneNumber(veh_edge_id) - 1
    original_lane_index = veh_lane_index
    veh_lane_index = min(max_lane_index, max(0, veh_lane_index + future_lateral_offset))
    vehicle_lane_id = veh_edge_id + f"_{veh_lane_index}"
    veh_lane_position = min(
        veh_lane_position,
        current_lane_length,
        traci.lane.getLength(vehicle_lane_id) - 0.1,
    )
    future_position = traci.simulation.convert2D(
        veh_edge_id, veh_lane_position, veh_lane_index
    )
    future_heading = traci.lane.getAngle(vehicle_lane_id, veh_lane_position)
    return future_position, future_heading


# @profile
def get_vehicle_info(veh_id, obs_dict, sumo_net):
    """Generate vehicle information for future trajectory prediction

    Args:
        veh_id (str): input vehicle id

    Returns:
        veh_info (dict): output dictionary of vehicle information
    """
    ego_obs = obs_dict["ego"]
    veh_info = VehicleInfoForPredict(
        id=veh_id,
        acceleration=ego_obs["acceleration"],
        route=traci.vehicle.getRoute(veh_id),
        route_index=traci.vehicle.getRouteIndex(veh_id),
        edge_id=ego_obs["edge_id"],
        lane_id=ego_obs["lane_id"],
        lane_index=ego_obs["lane_index"],
        position=traci.vehicle.getPosition(veh_id),
        velocity=ego_obs["velocity"],
        heading=ego_obs["heading"],
        lane_position=traci.vehicle.getLanePosition(veh_id),
        length=traci.vehicle.getLength(veh_id),
    )
    route_with_internal = sumolib.route.addInternal(sumo_net, veh_info.route)
    veh_info.route_id_list = [route._id for route in route_with_internal]
    # veh_info.route_length_list = [route._length for route in route_with_internal]
    veh_info.route_length_list = [
        traci.lane.getLength(edge_id + "_0") for edge_id in veh_info.route_id_list
    ]
    return veh_info


from dataclasses import dataclass
from typing import List, Optional


@dataclass
class VehicleInfoForPredict:
    id: str
    acceleration: float
    route: List[str]
    route_index: int
    edge_id: str
    lane_id: str
    lane_index: int
    position: List[float]
    velocity: float
    heading: float
    lane_position: float
    length: float
    route_id_list: Optional[List[str]] = None
    route_length_list: Optional[List[float]] = None

    def __getitem__(self, item):
        return self.__dict__[item]
