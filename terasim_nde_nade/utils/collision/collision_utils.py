from typing import Dict, Any, Tuple, Optional
from terasim.overlay import traci
from loguru import logger
from itertools import chain
from ..agents.vehicle import get_lane_angle
from .constants import (
    intersection_cutin_prob,
    intersection_neglect_conflict_lead_prob,
    intersection_rearend_prob,
    intersection_tfl_prob,
    intersection_headon_prob,
    roundabout_fail_to_yield_prob,
    roundabout_cutin_prob,
    roundabout_neglect_conflict_lead_prob,
    roundabout_rearend_prob,
    highway_cutin_prob,
    highway_rearend_prob,
)

# Cache for traffic light controlled lanes
tls_controlled_lane_set = None

def cache_tls_controlled_lane_set():
    """Cache the set of lanes controlled by traffic lights."""
    global tls_controlled_lane_set
    for tls_id in traci.trafficlight.getIDList():
        controlled_links = traci.trafficlight.getControlledLinks(tls_id)
        lane_set = {
            lane
            for link in chain.from_iterable(controlled_links)
            for lane in [link[0], link[-1]]
        }
        tls_controlled_lane_set.update(lane_set)

def get_distance_to_next_tls(veh_id: str) -> float:
    """Get the distance to the next traffic light for a vehicle."""
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
) -> str:
    """Determine the location type (highway, intersection, or roundabout) for a vehicle."""
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

def is_head_on(ego_obs: Dict[str, Any], leader_info: Optional[Tuple]) -> bool:
    """Check if two vehicles are in a head-on configuration."""
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

    return 120 < start_angle < 240

def get_collision_type_and_prob(
    obs_dict: Dict[str, Any],
    negligence_command: Any,
    location: Optional[str] = None,
) -> Tuple[float, str]:
    """
    Given current observation and the negligence mode, detect what type of collisions will be generated.
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