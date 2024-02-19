import numpy as np
import math

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
    local_observation = observation["local"].data
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
    local_observation = observation["local"].data
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

    intersection_cutin_prob = 1.0648925125333899e-04 * 0.10 * 0.61 * 1.5 * 2.9 * 0.5 * 0.73 * 1.38 * 1.05 * 0.86
    intersection_neglect_conflict_lead_prob = 5.5069126130818786e-05 * 2.67 * 0.61 * 1.39 * 0.89 * 1.57 * 0.7 * 1.48
    intersection_rearend_prob = 1.0149373787617288e-06 * 2.78 * 0.44 * 0.29 * 0.62 * 2 * 2 * 1.56 * 0.67
    intersection_tfl_prob = 8.797353696327892e-04 * 0.37 * 0.13 * 2.67 * 0.75 * 1.17 * 0.5 * 0.5 * 2 * 2
    intersection_headon_prob = 1.0113094565177164e-06 * 4.71 * 0.1 * 2.57 * 0.25 * 0.5 * 2 * 2 * 1.42 * 1.76

    roundabout_fail_to_yield_prob = 1e-7 * 10 * 2 * 2 * 2 * 2
    roundabout_cutin_prob = 4.570171651138106e-05 * 0.35 * 0.6 * 0.5 * 2 * 0.77 * 2
    roundabout_neglect_conflict_lead_prob = 2.3721857594970477e-05 * 0.1 * 0.33 * 0.2 * 2 * 2 * 2 * 2 * 2
    roundabout_rearend_prob = 4.884970702773788e-07 * 0.2 * 5 * 0.5 * 2 * 2 * 0.5
    
    highway_cutin_prob = 9.887380418491011e-05 * 0.232 * 1.65 * 1.23 * 0.36 * 2 * 1.27 * 0.5 * 1.34
    highway_rearend_prob = 1.0327637301820217e-04 * 3.64 * 1.21 * 1.63 * 2 * 2 * 2 * 0.5 * 0.5

    
    local_observation = observation["local"].data
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