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
    ego_veh_road_id = ego_veh_obs['road_id'] if ego_veh_obs else None
    lead_veh_road_id = lead_veh_obs['road_id'] if lead_veh_obs else None
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
    ego_veh_lane_id = ego_veh_obs['road_id'] + "_" + str(ego_veh_obs['lane_index']) if ego_veh_obs else None
    lead_veh_lane_id = lead_veh_obs['road_id'] + "_" + str(lead_veh_obs['lane_index']) if lead_veh_obs else None
    
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

    highway_cutin_prob = 7.2e-8 * 0.952 * 0.91 * 1.5 * 0.67 * 0.91 * 1.01 * 0.872 * 100 # some sideswipe, some angle
    highway_rearend_prob = 8.4e-5 * 0.748 * 0.46 * 1.5 * 1.3 * 1.5 * 0.71 * 0.77 * 0.76 * 1.15

    roundabout_rearend_prob = 3.6e-8 * 0.952 * 0.86 * 0.98 * 1.27 * 1.25 * 1.04 * 0.79 * 1.39
    roundabout_neglect_conflict_lead_prob = 3.6e-7 * 0.5 * 2.27 * 1.5 * 0.6 * 0.76 * 0.85 * 0.73 # ignore conflict
    roundabout_cutin_prob = 8.4e-8 * 1.121 * 0.91 * 1.2 * 0.93 * 1.03 * 1.01 * 0.97 * 100 # mostly sideswipe

    intersection_rearend_prob = 6e-7 * 0.85 * 1.14 * 0.888 * 1.05 * 1.03 * 1.07 * 0.93
    intersection_cutin_prob = 1.2e-5 * 0.804 * 1.31 * 0.888 * 1.05 * 0.6 * 1.01 * 0.85 * 1.17 # mostly sideswipe
    intersection_neglect_conflict_lead_prob = 1.2e-6 * 0.5 * 0.1 * 0.888 * 1.05 * 1.27 * 1.25 * 0.85 * 1.21 # ignore conflict
    intersection_tfl_prob = 1.2e-5 * 1.026 * 2.02 * 0.888 * 1.05 * 1.16 * 0.937 * 1.046# ignore traffic light
    intersection_headon_prob = 1.2e-7 * 1.388 * 0.888 * 1.05 * 0.74 * 0.97 * 1.76 * 0.726# head on

    local_observation = observation["local"].data
    ego_veh_obs = local_observation["Ego"]
    lead_veh_obs = local_observation["Lead"]
    ego_veh_lane_id = ego_veh_obs['road_id'] + "_" + str(ego_veh_obs['lane_index']) if ego_veh_obs else None
    lead_veh_lane_id = lead_veh_obs['road_id'] + "_" + str(lead_veh_obs['lane_index']) if lead_veh_obs else None

    rear_end = is_rear_end(observation)


    if "roundabout" in location_region or (neg_location_region is not None and "roundabout" in neg_location_region):
        if negligence_mode == "LeftFoll" or negligence_mode == "RightFoll":
            return roundabout_cutin_prob, "roundabout_cutin"
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