import numpy as np
from dataclasses import dataclass
from typing import List, Optional

from terasim.overlay import traci, profile
from terasim_nde_nade.vehicle.nde_vehicle_utils import CommandType


@dataclass
class VulnerableRoadUserInfoForPredict:
    id: str
    acceleration: float
    edge_id: str
    lane_id: str
    position: List[float]
    velocity: float
    heading: float
    length: float
    route_id_list: Optional[List[str]] = None
    route_length_list: Optional[List[float]] = None

    def __getitem__(self, item):
        return self.__dict__[item]


@profile
def get_vulnerbale_road_user_info(vru_id, obs_dict, sumo_net):
    """Generate vehicle information for future trajectory prediction

    Args:
        veh_id (str): input vehicle id

    Returns:
        veh_info (dict): output dictionary of vehicle information
    """
    ego_obs = obs_dict["ego"]
    vru_info = VulnerableRoadUserInfoForPredict(
        id=vru_id,
        acceleration=0,
        edge_id=traci.person.getRoadID(vru_id),
        lane_id=traci.person.getLaneID(vru_id),
        position=traci.person.getPosition(vru_id),
        velocity=traci.person.getSpeed(vru_id),
        heading=traci.person.getAngle(vru_id),
        length=traci.person.getLength(vru_id),
    )
    vru_info.route_id_list = [traci.person.getRoadID(vru_id)]
    # veh_info.route_length_list = [route._length for route in route_with_internal]
    vru_info.route_length_list = [
        traci.lane.getLength(edge_id + "_0") for edge_id in vru_info.route_id_list
    ]
    return vru_info


def predict_future_trajectory_vulnerable_road_user(modality, vru_info, control_command_dict, current_time):
    """Predict future trajectory of vulnerable road user in 0.5s time resolution.

    Args:
        modality (str): modality of the control command
        vru_info (dict): dictionary of vehicle information
        control_command_dict (dict): dictionary of control command
        current_time (float): current simulation time

    Returns:
        future_trajectory_array (np.array): future trajectory array
    """
    if modality == "normal":
        return None
    elif modality == "negligence":
        assert control_command_dict[modality].command_type == CommandType.TRAJECTORY
        future_trajectory_array = [[
                vru_info.position[0],
                vru_info.position[1],
                vru_info.heading,
                vru_info.velocity,
                0,
            ]]
        index_add = 5
        for i in range(5):
            if (i+1)*index_add-1 >= len(control_command_dict[modality].future_trajectory)-1:
                p = control_command_dict[modality].future_trajectory[-1]
                print("reach the end")
            else:
                p = control_command_dict[modality].future_trajectory[(i+1)*index_add-1]
            future_trajectory_array.append(
                [p[0], p[1], vru_info.heading, vru_info.velocity, (i+1)*index_add*0.1]
            )
        future_trajectory_array = np.array(future_trajectory_array)
        future_trajectory_array[:, -1] += current_time
        return future_trajectory_array
    else:
        print(f"unknown modality: {modality}")
        return None