import numpy as np
import math
from terasim.overlay import traci
import attr
import sumolib
from terasim.utils import (
    sumo_coordinate_to_center_coordinate,
    sumo_heading_to_orientation,
)

# from .nde_vehicle_utils_cython import get_future_position_on_route
from typing import List, Tuple, Optional
from enum import Enum
from collections import namedtuple
from scipy.interpolate import interp1d

# Define the TrajectoryPoint named tuple
TrajectoryPoint = namedtuple("TrajectoryPoint", ["timestep", "position", "heading"])
import jax.numpy as jnp
from .nde_vehicle_utils_cython import *
from numba import jit


class Command(Enum):
    DEFAULT = "default"
    LEFT = "left"
    RIGHT = "right"
    TRAJECTORY = "trajectory"
    ACC = "acceleration"


@attr.s
class NDECommand:
    """
    Represents a command for a vehicle in a Non-Deterministic Environment (NDE).
    if the command is "default", the vehicle will follow the SUMO controlled model, other elements will be ignored
    if the command is "left" or "right", the vehicle will change lane to the left or right, other elements will be ignored
    if the command is "trajectory", the vehicle will follow the future trajectory, which will be predicted according to the current acceleration, other elements will be ignored
    if the command is "acceleration", the vehicle will decelerate to stop using the acceleration element
    """

    command: Command = attr.ib(default=Command.DEFAULT, converter=Command)
    acceleration: float = attr.ib(default=0.0)
    future_trajectory: List[Tuple[float, float]] = attr.ib(factory=list)
    prob: float = attr.ib(default=1.0)
    duration: float = attr.ib(default=0.1)


def get_next_lane_edge(net, lane_id):
    origin_lane = net.getLane(lane_id)
    outgoing_lanes = [conn.getToLane() for conn in origin_lane.getOutgoing()]
    outgoing_edges = [lane.getEdge() for lane in outgoing_lanes]
    return outgoing_lanes[0].getID(), outgoing_edges[0].getID()


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


def is_in_lanes(lane_id, lanes):
    return lane_id in set(lanes)


def in_which_lane_group(lane_id, lane_groups):
    for index, lanes in lane_groups.items():
        if is_in_lanes(lane_id, lanes):
            return index
    return -1


def get_location(lane_id, lane_config):
    for lane_type, lane_groups in lane_config.items():
        index = in_which_lane_group(lane_id, lane_groups)
        if index != -1:
            return lane_type + "_" + str(index)
    return "intersection"


def is_rear_end(observation):
    from terasim.overlay import traci

    local_observation = observation["local"]
    ego_veh_obs = local_observation["Ego"]
    lead_veh_obs = local_observation["Lead"]
    ego_veh_road_id = ego_veh_obs["edge_id"] if ego_veh_obs else None
    lead_veh_road_id = lead_veh_obs["edge_id"] if lead_veh_obs else None
    ego_veh_route_id = (
        traci.vehicle.getRouteID(ego_veh_obs["veh_id"]) if ego_veh_obs else None
    )
    lead_veh_route_id = (
        traci.vehicle.getRouteID(lead_veh_obs["veh_id"]) if lead_veh_obs else None
    )
    ego_veh_route = traci.vehicle.getRoute(ego_veh_obs["veh_id"]) if ego_veh_obs else []
    lead_veh_route = (
        traci.vehicle.getRoute(lead_veh_obs["veh_id"]) if lead_veh_obs else []
    )

    if (ego_veh_obs and lead_veh_obs) and (
        (ego_veh_road_id == lead_veh_road_id)
        or (ego_veh_route_id == lead_veh_route_id)
        or (ego_veh_road_id in lead_veh_route)
        or (lead_veh_road_id in ego_veh_route)
    ):
        return True
    else:
        return False


def get_lane_angle(lane_id, mode="start"):
    from terasim.overlay import traci

    if lane_id is None:
        return None
    lane_shape = traci.lane.getShape(lane_id)
    if len(lane_shape) < 2:
        return None

    x1, y1 = lane_shape[0]
    x2, y2 = lane_shape[1] if mode == "start" else lane_shape[-1]

    # Calculate the angle using trigonometry
    delta_x = x2 - x1
    delta_y = y2 - y1
    angle_radians = math.atan2(delta_y, delta_x)

    # Convert radians to degrees
    angle_degrees = math.degrees(angle_radians)

    # Normalize the angle to be between 0 and 360 degrees
    if angle_degrees < 0:
        angle_degrees += 360

    return angle_degrees


def is_head_on(observation):
    local_observation = observation["local"]
    ego_veh_obs = local_observation["Ego"]
    lead_veh_obs = local_observation["Lead"]
    ego_veh_lane_id = ego_veh_obs["lane_id"] if ego_veh_obs else None
    lead_veh_lane_id = lead_veh_obs["lane_id"] if lead_veh_obs else None

    ego_veh_lane_start_angle = get_lane_angle(ego_veh_lane_id)
    lead_veh_lane_start_angle = get_lane_angle(lead_veh_lane_id)

    ego_veh_lane_end_angle = get_lane_angle(ego_veh_lane_id, mode="end")
    lead_veh_lane_end_angle = get_lane_angle(lead_veh_lane_id, mode="end")

    start_angle = None
    end_angle = None
    if (ego_veh_lane_start_angle is not None) and (
        lead_veh_lane_start_angle is not None
    ):
        start_angle = abs(ego_veh_lane_start_angle - lead_veh_lane_start_angle)
    if (ego_veh_lane_end_angle is not None) and (lead_veh_lane_end_angle is not None):
        end_angle = abs(ego_veh_lane_end_angle - lead_veh_lane_end_angle)

    if (
        (start_angle is not None)
        and (end_angle is not None)
        and (120 < start_angle < 240)
        and (120 < end_angle < 240)
    ):
        return True
    return False


def get_collision_type_and_prob(
    observation, negligence_mode, location_region, neg_location_region
):
    """
    Given current observation and the negligence mode, detect what type of collisions will be generated
    """

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
        4.570171651138106e-05
        * 0.35
        * 0.6
        * 0.5
        * 2
        * 0.77
        * 2
        * 0.63
        * 1.34
        * 1.2
        * 0.9
    )
    roundabout_neglect_conflict_lead_prob = (
        2.3721857594970477e-05
        * 0.1
        * 0.33
        * 0.2
        * 2
        * 2
        * 2
        * 2
        * 2
        * 0.5
        * 0.63
        * 1.19
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

    local_observation = observation["local"]
    ego_veh_obs = local_observation["Ego"]
    lead_veh_obs = local_observation["Lead"]
    ego_veh_lane_id = ego_veh_obs["lane_id"] if ego_veh_obs else None
    lead_veh_lane_id = lead_veh_obs["lane_id"] if lead_veh_obs else None

    rear_end = is_rear_end(observation)
    if "roundabout" in location_region or (
        neg_location_region is not None and "roundabout" in neg_location_region
    ):
        if negligence_mode == "LeftFoll" or negligence_mode == "RightFoll":
            return roundabout_cutin_prob, "roundabout_cutin"
        elif negligence_mode == "RDBT_FAIL_TO_YIELD":
            return roundabout_fail_to_yield_prob, "roundabout_fail_to_yield"
        elif rear_end:
            return roundabout_rearend_prob, "roundabout_rearend"
        else:
            return (
                roundabout_neglect_conflict_lead_prob,
                "roundabout_neglect_conflict_lead",
            )
    elif "highway" in location_region:
        if negligence_mode == "LeftFoll" or negligence_mode == "RightFoll":
            return highway_cutin_prob, "highway_cutin"
        else:
            return highway_rearend_prob, "highway_rearend"
    elif "intersection" in location_region:
        if negligence_mode == "LeftFoll" or negligence_mode == "RightFoll":
            return intersection_cutin_prob, "intersection_cutin"
        elif rear_end:
            return intersection_rearend_prob, "intersection_rearend"
        elif is_head_on(observation):
            return intersection_headon_prob, "intersection_headon"
        elif negligence_mode == "TFL":
            return intersection_tfl_prob, "intersection_tfl"
        else:
            return (
                intersection_neglect_conflict_lead_prob,
                "intersection_neglect_conflict_lead",
            )
    else:
        return 0, "no_collision"


def predict_future_distance(
    velocity: float,
    acceleration: float,
    duration_array: np.ndarray,
    max_velocity: float,
) -> np.ndarray:
    """Predict the future distance of the vehicle.

    Args:
        velocity (float): the velocity of the vehicle
        acceleration (float): the acceleration of the vehicle
        duration_array (np.ndarray): the array of duration

    Returns:
        future_distance_array (np.ndarray): the array of future distance
    """
    future_distance_array = (
        velocity * duration_array + 0.5 * acceleration * duration_array**2
    )
    # the lane_position delta list should be non-decreasing and non-negative
    future_distance_array = np.maximum.accumulate(future_distance_array.clip(min=0))
    return future_distance_array


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
        if control_command.command == Command.ACC
        else veh_info["acceleration"]
    )
    max_velocity = traci.vehicle.getAllowedSpeed(veh_id)
    future_distance_array = predict_future_distance(
        veh_info["velocity"], acceleration, duration_array, max_velocity
    )
    lateral_offset = 0
    if control_command.command == Command.LEFT:
        lateral_offset = 1
    elif control_command.command == Command.RIGHT:
        lateral_offset = -1

    trajectory_array = np.array(
        [veh_info.position[0], veh_info.position[1], veh_info.heading, 0]
    )

    lanechange_finish_trajectory_point = None
    if (
        control_command.command == Command.LEFT
        or control_command.command == Command.RIGHT
    ):
        lanechange_finish_timestep = np.argmin(
            np.abs(duration_array - control_command.duration)
        )
        lanechange_finish_position, lanechange_finish_final_heading = (
            get_future_position_on_route(
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
                duration_array[lanechange_finish_timestep],
            ]
        )
        trajectory_array = np.vstack(
            (trajectory_array, lanechange_finish_trajectory_point)
        )

    for duration, distance in zip(duration_array[1:], future_distance_array[1:]):
        if (
            lanechange_finish_trajectory_point is not None
            and duration <= lanechange_finish_trajectory_point[3]
        ):
            continue
        future_position, future_heading = get_future_position_on_route(
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
                    [future_position[0], future_position[1], future_heading, duration]
                ),
            )
        )

    future_trajectory_array = interpolate_future_trajectory(
        trajectory_array, interpolate_resolution
    )

    future_trajectory_array[:, 3] += current_time
    return future_trajectory_array


#  @profile
# def interpolate_future_trajectory(
#         trajectory_list_array: np.ndarray,
#         interpolate_resolution: float,
#     ) -> np.ndarray:
#     """
#     Given the initial, final, and several intermediate trajectory points, interpolate resolution, interpolate the future trajectory.

#     The trajectory point is [x, y, heading, time], and the interpolate_resolution is the time resolution of the interpolation.
#     The trajectory list array is composed of multiple trajectory points, which must include the first and last trajectory points.
#     """
#     # Extract the time and position values
#     time_values = trajectory_list_array[:, 3]
#     position_values = trajectory_list_array[:, :3]

#     # Create the interpolation function
#     interpolation_function = interp1d(time_values, position_values, axis=0, kind='linear')

#     # Create the new time values
#     new_time_values = np.arange(time_values[0], time_values[-1], interpolate_resolution)

#     # Interpolate the position values
#     new_position_values = interpolation_function(new_time_values)

#     # Combine the new time and position values
#     new_trajectory_list_array = np.hstack((new_position_values, new_time_values[:, None]))

#     return new_trajectory_list_array


def get_future_position_on_route(
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
    veh_lane_position = min(veh_lane_position, current_lane_length)
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
    # assert each element in veh_info.route_length_list is equal to the corresponding element in lane_based_route_length_list
    # assert all([abs(a - b) < 1e-4 for a, b in zip(veh_info.route_length_list, lane_based_route_length_list)])
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


# @lru_cache(maxsize=256)
# def get_route_with_internal(net, route):
#     return sumolib.route.addInternal(net, route)

# def sumo_trajectory_to_normal_trajectory(sumo_trajectory, veh_length=5.0):
#     """Convert sumo trajectory to normal trajectory. Both trajectories are nparrays

#     sumo trajectory is defined as [x,y,heading,timestamp], where x, y denotes the position of the front bumper, heading start from north and goes clockwise (degrees), timestamp is the time step.

#     normal trajectory is defined as [x,y,orientation,timestamp], where x, y denotes the position of the center of the vehicle, orientation follows radians, timestamp is the time step.

#     """
#     normal_trajectory = np.zeros(sumo_trajectory.shape)
#     normal_trajectory[:, 2] = np.arctan2(np.sin(np.radians(90 - sumo_trajectory[:, 2])), np.cos(np.radians(90 - sumo_trajectory[:, 2])))

#     normal_trajectory[:, 0] = sumo_trajectory[:, 0] - veh_length / 2 * np.cos(normal_trajectory[:, 2])
#     normal_trajectory[:, 1] = sumo_trajectory[:, 1] - veh_length / 2 * np.sin(normal_trajectory[:, 2])
#     if sumo_trajectory.shape[1] > 3:
#         normal_trajectory[:, 3:] = sumo_trajectory[:, 3:]
#     return normal_trajectory

# @profile
# def collision_check(traj1: np.ndarray, traj2: np.ndarray, veh_length: float, tem_len: float, circle_r: float):

#     traj1 = sumo_trajectory_to_normal_trajectory(traj1, veh_length)
#     traj2 = sumo_trajectory_to_normal_trajectory(traj2, veh_length)

#     # Get the unique time steps
#     time_steps = np.unique(np.concatenate((traj1[:, 3], traj2[:, 3])))

#     for time_step in time_steps:
#         # Get the positions at the current time step
#         traj_point1 = traj1[traj1[:, 3] == time_step].flatten()
#         traj_point2 = traj2[traj2[:, 3] == time_step].flatten()

#         # Get the circle center lists
#         center_list_1 = get_circle_center_list(traj_point1, veh_length, tem_len)
#         center_list_2 = get_circle_center_list(traj_point2, veh_length, tem_len)

#         # Check for collisions
#         for p1 in center_list_1:
#             for p2 in center_list_2:
#                 dist = np.linalg.norm(p1 - p2)
#                 if dist <= 2 * circle_r:
#                     return True, time_step
#     return False, None

# def get_circle_center_list(traj_point: np.ndarray, veh_length: float, tem_len: float):
#     center1 = (traj_point[0], traj_point[1])
#     heading = traj_point[2]
#     center0 = (
#         center1[0] + (veh_length / 2 - tem_len) * np.cos(heading),
#         center1[1] + (veh_length / 2 - tem_len) * np.sin(heading)
#     )
#     center2 = (
#         center1[0] - (veh_length / 2 - tem_len) * np.cos(heading),
#         center1[1] - (veh_length / 2 - tem_len) * np.sin(heading)
#     )
#     center_list = [center0, center1, center2]
#     return np.array(center_list)
