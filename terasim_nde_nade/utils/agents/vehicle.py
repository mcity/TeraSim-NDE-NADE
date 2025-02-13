import math
from terasim.overlay import traci
import sumolib
from typing import List, Tuple, Dict, Any, Optional, Callable
from dataclasses import dataclass
from terasim_nde_nade.utils.agents.base import AgentInfo
from collections import namedtuple


# Define the TrajectoryPoint named tuple
TrajectoryPoint = namedtuple("TrajectoryPoint", ["timestep", "position", "heading"])

@dataclass
class VehicleInfoForPredict(AgentInfo):
    """Vehicle information for trajectory prediction."""
    acceleration: float
    route: List[str]
    route_index: int
    edge_id: str
    lane_id: str
    lane_index: int
    lane_position: float
    length: float
    route_id_list: Optional[List[str]] = None
    route_length_list: Optional[List[float]] = None
    upcoming_lane_id_list: Optional[List[str]] = None

    def __getitem__(self, item):
        return self.__dict__[item]

def get_next_lane_edge(net, lane_id):
    """Get the next lane and edge IDs for a given lane."""
    origin_lane = net.getLane(lane_id)
    outgoing_lanes = [conn.getToLane() for conn in origin_lane.getOutgoing()]
    outgoing_edges = [lane.getEdge() for lane in outgoing_lanes]
    return outgoing_lanes[0].getID(), outgoing_edges[0].getID()

def get_lane_angle(lane_id: str, mode: str = "start") -> float:
    """Get the angle of a lane at a specific position."""
    if mode == "start":
        relative_position = 0
    elif mode == "end":
        relative_position = traci.lane.getLength(lane_id) - 0.1
    else:
        raise ValueError("mode must be either start or end")
    lane_angle = traci.lane.getAngle(lane_id, relative_position)
    return lane_angle

def get_lanechange_longitudinal_speed(
    veh_id: str, 
    current_speed: float, 
    lane_width: Optional[float] = None, 
    lanechange_duration: float = 1.0
) -> float:
    """Calculate the longitudinal speed during a lane change maneuver."""
    if lane_width is None:
        lane_width = traci.lane.getWidth(traci.vehicle.getLaneID(veh_id))
    lateral_speed = lane_width / lanechange_duration
    return math.sqrt(max(current_speed**2 - lateral_speed**2, 0))

def get_upcoming_lane_id_list(veh_id: str) -> List[str]:
    """Get a list of upcoming lane IDs for a vehicle."""
    veh_next_links = traci.vehicle.getNextLinks(veh_id)
    current_lane_id = traci.vehicle.getLaneID(veh_id)
    lane_links = traci.lane.getLinks(current_lane_id)
    upcoming_lane_id_list = [current_lane_id]
    
    if isinstance(lane_links, list) and len(lane_links) > 0:
        for lane_link in lane_links:
            lane_id = lane_link[0]
            via_lane_id = lane_link[4]
            if via_lane_id != "":
                upcoming_lane_id_list.append(via_lane_id)
            upcoming_lane_id_list.append(lane_id)

    if len(veh_next_links) == 0:
        return upcoming_lane_id_list
        
    for link in veh_next_links:
        lane_id = link[0]
        via_lane_id = link[4]
        upcoming_lane_id_list.append(via_lane_id)
        upcoming_lane_id_list.append(lane_id)
    return upcoming_lane_id_list

def get_vehicle_info(veh_id: str, obs_dict: dict, sumo_net) -> VehicleInfoForPredict:
    """Generate vehicle information for future trajectory prediction."""
    veh_info = VehicleInfoForPredict(
        id=veh_id,
        acceleration=traci.vehicle.getAcceleration(veh_id),
        route=traci.vehicle.getRoute(veh_id),
        route_index=traci.vehicle.getRouteIndex(veh_id),
        edge_id=traci.vehicle.getRoadID(veh_id),
        lane_id=traci.vehicle.getLaneID(veh_id),
        lane_index=traci.vehicle.getLaneIndex(veh_id),
        position=traci.vehicle.getPosition(veh_id),
        velocity=traci.vehicle.getSpeed(veh_id),
        heading=traci.vehicle.getAngle(veh_id),
        lane_position=traci.vehicle.getLanePosition(veh_id),
        length=traci.vehicle.getLength(veh_id),
    )
    
    route_with_internal = sumolib.route.addInternal(sumo_net, veh_info.route)
    veh_info.route_id_list = [route._id for route in route_with_internal]
    veh_info.route_length_list = [
        traci.lane.getLength(edge_id + "_0") for edge_id in veh_info.route_id_list
    ]
    veh_info.upcoming_lane_id_list = get_upcoming_lane_id_list(veh_id)
    return veh_info

def is_car_following(follow_id: str, leader_id: str) -> bool:
    """Check if one vehicle is following another vehicle."""
    current_edge_id = traci.vehicle.getLaneID(follow_id)
    leader_edge_id = traci.vehicle.getLaneID(leader_id)
    current_angle = traci.vehicle.getAngle(follow_id)
    leader_angle = traci.vehicle.getAngle(leader_id)
    
    # Check if vehicles are on the same link
    if current_edge_id == leader_edge_id:
        return True
    elif abs((current_angle - leader_angle + 180) % 360 - 180) <= 5:
        return True
    
    # Check future links
    follower_future_link_infos = traci.vehicle.getNextLinks(follow_id)
    if len(follower_future_link_infos) == 0:
        return False
        
    follower_future_lane_id = follower_future_link_infos[0][0]
    follower_future_junction_lane_id = follower_future_link_infos[0][4]
    
    if (leader_edge_id in follower_future_lane_id or 
        leader_edge_id in follower_future_junction_lane_id):
        return True

    leader_future_link_infos = traci.vehicle.getNextLinks(leader_id)
    if len(leader_future_link_infos) == 0:
        return False
        
    leader_future_lane_id = leader_future_link_infos[0][0]
    leader_junction_lane_id = leader_future_link_infos[0][4]
    
    # Check if vehicles share any future links
    if len(set([
        follower_future_lane_id,
        follower_future_junction_lane_id,
        leader_future_lane_id,
        leader_junction_lane_id,
    ])) < 4:
        return True
        
    return False
