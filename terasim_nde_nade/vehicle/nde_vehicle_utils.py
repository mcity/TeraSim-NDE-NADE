import numpy as np
import math
from terasim.overlay import traci
import attr
import sumolib
from terasim.utils import sumo_coordinate_to_center_coordinate, sumo_heading_to_orientation
from typing import List, Tuple, Optional

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
    ego_veh_route_id = traci.vehicle.getRouteID(ego_veh_obs['veh_id']) if ego_veh_obs else None
    lead_veh_route_id = traci.vehicle.getRouteID(lead_veh_obs['veh_id']) if lead_veh_obs else None
    ego_veh_route = traci.vehicle.getRoute(ego_veh_obs['veh_id']) if ego_veh_obs else []
    lead_veh_route = traci.vehicle.getRoute(lead_veh_obs['veh_id']) if lead_veh_obs else []

    if (ego_veh_obs and lead_veh_obs) and ((ego_veh_road_id == lead_veh_road_id) or (ego_veh_route_id == lead_veh_route_id) or \
            (ego_veh_road_id in lead_veh_route) or (lead_veh_road_id in ego_veh_route)):
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
    if (ego_veh_lane_start_angle is not None) and (lead_veh_lane_start_angle is not None):
        start_angle = abs(ego_veh_lane_start_angle - lead_veh_lane_start_angle)
    if (ego_veh_lane_end_angle is not None) and (lead_veh_lane_end_angle is not None):
        end_angle = abs(ego_veh_lane_end_angle - lead_veh_lane_end_angle)
        
    if (start_angle is not None) and (end_angle is not None) and (120 < start_angle < 240) and (120 < end_angle < 240):
        return True
    return False
    

def get_collision_type_and_prob(observation, negligence_mode, location_region, neg_location_region):
    """
    Given current observation and the negligence mode, detect what type of collisions will be generated
    """

    intersection_cutin_prob = 1.0648925125333899e-04 * 0.10 * 0.61 * 1.5 * 2.9 * 0.5 * 0.73 * 1.38 * 1.05 * 0.86 * 0.87
    intersection_neglect_conflict_lead_prob = 5.5069126130818786e-05 * 2.67 * 0.61 * 1.39 * 0.89 * 1.57 * 0.7 * 1.48 * 0.82 * 0.8 * 2.35 * 0.6 * 0.8 * 0.5
    intersection_rearend_prob = 1.0149373787617288e-06 * 2.78 * 0.44 * 0.29 * 0.62 * 2 * 2 * 1.56 * 0.67 * 2 * 0.5 * 1.5 * 1.5 * 1.5 * 0.7
    intersection_tfl_prob = 8.797353696327892e-04 * 0.37 * 0.13 * 2.67 * 0.75 * 1.17 * 0.5 * 0.5 * 2 * 2 * 0.5 * 0.5 * 10 * 5 * 1.23 * 1.2
    intersection_headon_prob = 1.0113094565177164e-06 * 4.71 * 0.1 * 2.57 * 0.25 * 0.5 * 2 * 2 * 1.42 * 1.76 * 2 * 2 * 1.74 * 0.8 * 1.32 * 0.8

    roundabout_fail_to_yield_prob = 1e-7 * 10 * 2 * 2 * 2 * 2 * 0.9 * 1.1 * 0.81
    roundabout_cutin_prob = 4.570171651138106e-05 * 0.35 * 0.6 * 0.5 * 2 * 0.77 * 2 * 0.63 * 1.34 * 1.2 * 0.9
    roundabout_neglect_conflict_lead_prob = 2.3721857594970477e-05 * 0.1 * 0.33 * 0.2 * 2 * 2 * 2 * 2 * 2 * 0.5 * 0.63 * 1.19
    roundabout_rearend_prob = 4.884970702773788e-07 * 0.2 * 5 * 0.5 * 2 * 2 * 0.5 * 0.84 * 0.5 * 1.4 * 0.8
    
    highway_cutin_prob = 9.887380418491011e-05 * 0.232 * 1.65 * 1.23 * 0.36 * 2 * 1.27 * 0.5 * 1.34 * 2 * 0.5 * 0.75 * 1.1 * 1.1
    highway_rearend_prob = 1.0327637301820217e-04 * 3.64 * 1.21 * 1.63 * 2 * 2 * 2 * 0.5 * 0.5 * 1.4 * 0.83 * 0.81 * 0.92 * 1.15

    
    local_observation = observation["local"]
    ego_veh_obs = local_observation["Ego"]
    lead_veh_obs = local_observation["Lead"]
    ego_veh_lane_id = ego_veh_obs["lane_id"] if ego_veh_obs else None
    lead_veh_lane_id = lead_veh_obs["lane_id"] if lead_veh_obs else None

    rear_end = is_rear_end(observation)
    if "roundabout" in location_region or (neg_location_region is not None and "roundabout" in neg_location_region):
        if negligence_mode == "LeftFoll" or negligence_mode == "RightFoll":
            return roundabout_cutin_prob, "roundabout_cutin"
        elif negligence_mode == "RDBT_FAIL_TO_YIELD":
            return roundabout_fail_to_yield_prob, "roundabout_fail_to_yield"
        elif rear_end:
            return roundabout_rearend_prob, "roundabout_rearend"
        else:
            return roundabout_neglect_conflict_lead_prob, "roundabout_neglect_conflict_lead"
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
            return intersection_neglect_conflict_lead_prob, "intersection_neglect_conflict_lead"
    else:
        return 0, "no_collision"
    

def predict_future_distance(velocity: float, acceleration: float, duration_array: np.ndarray) -> np.ndarray:
    """Predict the future distance of the vehicle.

    Args:
        velocity (float): the velocity of the vehicle
        acceleration (float): the acceleration of the vehicle
        duration_array (np.ndarray): the array of duration

    Returns:
        future_distance_array (np.ndarray): the array of future distance
    """
    future_distance_array = velocity * duration_array + 0.5 * acceleration * duration_array ** 2
    # the lane_position delta list should be non-decreasing and non-negative
    future_distance_array = np.maximum.accumulate(future_distance_array.clip(min=0))
    return future_distance_array

def predict_future_trajectory(veh_id, obs_dict, control_command, sumo_net, time_horizon_step=20, time_resolution=0.1, lanechange_duration=1.0):
    """Predict the future trajectory of the vehicle.

    Args:
        veh_id (str): the id of the vehicle
        time_horizon_step (int): the time horizon of the prediction
        time_resolution (float): the time resolution of the prediction

    Returns:
        future_trajectory_dict (dict): the future trajectory of the vehicle
    """
    veh_info = get_vehicle_info(veh_id, obs_dict, sumo_net)
    # include the original position
    duration_array = np.array([time_horizon_id*time_resolution for time_horizon_id in range(time_horizon_step+1)])
    future_distance_list = predict_future_distance(veh_info["velocity"], veh_info["acceleration"], duration_array)
    center_position = sumo_coordinate_to_center_coordinate(veh_info.position[0], veh_info.position[1], sumo_heading_to_orientation(veh_info['heading']), veh_info["length"])
    lateral_offset = 0
    if control_command["command"] == "LEFT":
        lateral_offset = 1
    elif control_command["command"] == "RIGHT":
        lateral_offset = -1

    initial_trajectory_point = (center_position, veh_info["heading"], 0)

    # predict the final_position of the vehicle
    final_center_position, final_heading = get_future_position_on_route(veh_info["edge_id"], veh_info["lane_position"], veh_info["lane_index"], veh_info["lane_id"], veh_info["route_id_list"], veh_info["route_length_list"], future_distance_list[-1], lateral_offset)
    final_timestep = len(duration_array) - 1
    final_trajectory_point = (final_center_position, final_heading, final_timestep)

    lanechange_finish_trajectory_point = None
    if control_command["command"] == "LEFT" or control_command["command"] == "RIGHT":
        lanechange_finish_timestep = np.argmin(np.abs(duration_array - control_command["duration"]))
        lane_change_finished_distance = future_distance_list[lanechange_finish_timestep]
        lanechange_finish_center_position, lanechange_finish_final_heading = get_future_position_on_route(veh_info["edge_id"], veh_info["lane_position"], veh_info["lane_index"], veh_info["lane_id"], veh_info["route_id_list"], veh_info["route_length_list"], lane_change_finished_distance, lateral_offset)
        lanechange_finish_trajectory_point = (lanechange_finish_center_position, lanechange_finish_final_heading, lanechange_finish_timestep)
    
    future_trajectory = interpolate_future_trajectory(initial_trajectory_point, final_trajectory_point, duration_array, lanechange_finish_trajectory_point)
    return future_trajectory



def interpolate_future_trajectory(
        initial_trajectory_point: Tuple[Tuple[float, float], float, float], 
        final_trajectory_point: Tuple[Tuple[float, float], float, float], 
        duration_array: np.ndarray, 
        lc_finished_trajectory_point: Optional[Tuple[Tuple[float, float], float, float]] = None,
    ) -> List[Tuple[Tuple[float, float], float, float]]:
    """
    Given the initial, final, and lane change finished trajectory points, and a numpy array of durations, interpolate the future trajectory.
    """
    initial_position, initial_heading, initial_time = initial_trajectory_point
    final_position, final_heading, final_time = final_trajectory_point

    future_trajectory = []

    for duration in duration_array:
        if lc_finished_trajectory_point is not None:
            lc_position, lc_heading, lc_time = lc_finished_trajectory_point
            if duration <= lc_time:
                # Interpolate between initial and lc_finished_trajectory_point
                t = duration / lc_time
                position = tuple(np.array(initial_position) * (1 - t) + np.array(lc_position) * t)
                heading = initial_heading * (1 - t) + lc_heading * t
            else:
                # Interpolate between lc_finished_trajectory_point and final_trajectory_point
                t = (duration - lc_time) / (final_time - lc_time)
                position = tuple(np.array(lc_position) * (1 - t) + np.array(final_position) * t)
                heading = lc_heading * (1 - t) + final_heading * t
        else:
            # Interpolate between initial and final_trajectory_point
            t = duration / final_time
            position = tuple(np.array(initial_position) * (1 - t) + np.array(final_position) * t)
            heading = initial_heading * (1 - t) + final_heading * t
        future_trajectory.append((position, heading, duration))
    return future_trajectory

def get_future_position_on_route(
        veh_edge_id: str, 
        veh_lane_position: float, 
        veh_lane_index: int, 
        veh_lane_id: str,
        veh_route_id_list: List[str], 
        veh_route_length_list: List[float], 
        future_distance: float, 
        future_lateral_offset: float
    ) -> Tuple[Tuple[float, float], float]:
    """
    Given the current vehicle edge id, lane position, current lane id, and the future distance / future lateral offset, predict the future position of the vehicle.
    """
    veh_lane_position += future_distance
    current_lane_length = traci.lane.getLength(veh_lane_id)
    current_route_index = veh_route_id_list.index(veh_edge_id)

    # calculate the corresponding edge and lane position
    while veh_lane_position > current_lane_length and current_route_index < len(veh_route_id_list) - 1:
        current_route_index += 1
        veh_edge_id = veh_route_id_list[current_route_index]
        veh_lane_position -= current_lane_length
        current_lane_length = veh_route_length_list[current_route_index]

    # calculate the new lane index
    max_lane_index = traci.edge.getLaneNumber(veh_edge_id)
    veh_lane_index = min(max_lane_index, max(0, veh_lane_index + future_lateral_offset))
    vehicle_lane_id = veh_edge_id + f"_{veh_lane_index}"
    future_position = traci.simulation.convert2D(veh_edge_id, veh_lane_position, veh_lane_index)
    future_heading = traci.lane.getAngle(vehicle_lane_id, veh_lane_position)
    return future_position, future_heading

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
        route=traci.vehicle.getRoute(veh_id),
        route_index=traci.vehicle.getRouteIndex(veh_id),
        edge_id=ego_obs["edge_id"],
        lane_id=ego_obs["lane_id"],
        lane_index=ego_obs["lane_index"],
        position=traci.vehicle.getPosition(veh_id),
        velocity=ego_obs["velocity"],
        heading=ego_obs["heading"],
        lane_position=ego_obs["lane_position"],
        length=traci.vehicle.getLength(veh_id)
    )
    route_with_internal = sumolib.route.addInternal(sumo_net, veh_info['route'])
    veh_info.route_id_list = [route._id for route in route_with_internal]
    veh_info.route_length_list = [route._length for route in route_with_internal]
    return veh_info

@attr.s
class VehicleInfoForPredict:
    id: str = attr.ib()
    route: list = attr.ib()
    route_index: int = attr.ib()
    edge_id: str = attr.ib()
    lane_id: str = attr.ib()
    lane_index: int = attr.ib()
    position: list = attr.ib()
    velocity: float = attr.ib()
    heading: float = attr.ib()
    lane_position: float = attr.ib()
    length: float = attr.ib()
    route_id_list: list = attr.ib(default=None)
    route_length_list: list = attr.ib(default=None)

# @lru_cache(maxsize=256)
# def get_route_with_internal(net, route):
#     return sumolib.route.addInternal(net, route)