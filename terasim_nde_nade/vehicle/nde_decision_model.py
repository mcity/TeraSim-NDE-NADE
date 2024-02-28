from terasim.vehicle.decision_models.idm_model import IDMModel
import terasim.utils as utils
import numpy as np
import logging
from terasim.overlay import traci
import os
import json
import terasim_nde_nade.vehicle.nde_vehicle_utils as nde_utils
from terasim_nde_nade.vehicle.nde_vehicle_utils import get_collision_type_and_prob
import attr
from enum import Enum
from typing import List, Tuple
from addict import Dict

@attr.s
class Command(Enum):
    DEFAULT = "default"
    LEFT = "left"
    RIGHT = "right"
    TRAJECTORY = "trajectory"
    STOP = "stop"

@attr.s
class NDECommand:
    """
    Represents a command for a vehicle in a Non-Deterministic Environment (NDE).
    if the command is "default", the vehicle will follow the SUMO controlled model, other elements will be ignored
    if the command is "left" or "right", the vehicle will change lane to the left or right, other elements will be ignored
    if the command is "trajectory", the vehicle will follow the future trajectory, which will be predicted according to the current acceleration, other elements will be ignored
    if the command is "stop", the vehicle will decelerate to stop using the acceleration element
    """
    command: Command = attr.ib(default=Command.DEFAULT, converter=Command)
    acceleration: float = attr.ib(default=0.0)
    future_trajectory: List[Tuple[float, float]] = attr.ib(factory=list)
    prob: float = attr.ib(default=1.0)
    duration: float = attr.ib(default=0.1)

def get_location(veh_road_id, lane_config):
    return "intersection"

class NDEDecisionModel(IDMModel):
    
    def __init__(self, MOBIL_lc_flag=True, stochastic_acc_flag=False, IDM_parameters=None, MOBIL_parameters=None):
        current_path = os.path.dirname(os.path.abspath(__file__))
        lane_config_path = os.path.join(current_path, "lane_config.json")
        self.lane_config = json.load(open(lane_config_path, 'r'))
        super().__init__(MOBIL_lc_flag, stochastic_acc_flag, IDM_parameters, MOBIL_parameters)

    def get_negligence_prob(self, obs_dict, negligence_mode):  
        veh_road_id = obs_dict["ego"]["edge_id"]
        location = get_location(veh_road_id, self.lane_config)
        neg_veh_road_id, neg_location = None, None
        if obs_dict["local"].get(negligence_mode, None) is not None:
            neg_veh_road_id = obs_dict["local"][negligence_mode].get("edge_id", None)
            neg_location = get_location(neg_veh_road_id, self.lane_config)
        collision_prob, collision_type = get_collision_type_and_prob(obs_dict, negligence_mode, location, neg_location)
        return collision_prob, collision_type
    
    def change_vehicle_type_according_to_location(self, veh_id, vehicle_location):
        if "highway" in vehicle_location: # highway/freeway scenario
            traci.vehicle.setType(veh_id, "NDE_HIGHWAY")
        elif "intersection" in vehicle_location or "roundabout" in vehicle_location: # urban scenario
            traci.vehicle.setType(veh_id, "NDE_URBAN")
        else:
            raise ValueError(f"location {vehicle_location} not supported")
        
    def derive_control_command_from_observation(self, obs_dict):
        # change the IDM and MOBIL parameters based on the location
        vehicle_location = get_location(obs_dict["ego"]["edge_id"], self.lane_config)
        # self.change_vehicle_type_according_to_location(obs_dict["ego"]["veh_id"], vehicle_location)

        negligence_command_dict = Dict()

        lc_negligence_command_dict = Dict(self.lane_change_negligence(obs_dict))

        leader_info = traci.vehicle.getLeader(obs_dict["ego"]["veh_id"], 40)



        current_acceleration = obs_dict["ego"]["acceleration"]
        ff_acceleration = self.get_ff_acceleration(obs_dict)

        leader_id, leader_distance = None, None
        

        if leader_info is not None:
            leader_id, leader_distance = leader_info

        if leader_id is not None:
            cf_speed_with_leading_vehicle = traci.vehicle.getFollowSpeed(obs_dict["ego"]["veh_id"], obs_dict["ego"]["velocity"], leader_distance, traci.vehicle.getSpeed(leader_id), 7.06)
            cf_acceleration = (cf_speed_with_leading_vehicle - obs_dict["ego"]["velocity"]) / traci.simulation.getDeltaT()

        # no leading vehicle or controlled by traffic rules even with leading vehicle
        if leader_id is None or abs(cf_acceleration - current_acceleration) > 0.5:
            if ff_acceleration - cf_acceleration > 1:
                # the vehicle is constained by the traffic rules
                # print("traffic rule violation potential!")
                negligence_mode_set.add("traffic_rule")
                # highlight the vehicle with blue
                traci.vehicle.highlight(obs_dict["ego"]["veh_id"], (0, 0, 255, 255), 0.5, duration=0.3)
 
        if leader_id is not None:
            

        return None, None
    
    def traffic_rule_negligence(self, obs_dict):
        negligence_command_dict = Dict()
        return negligence_command_dict
    
    def leader_negligence(self, obs_dict, ff_acceleration, cf_acceleration):
        negligence_command_dict = {}
        if ff_acceleration - cf_acceleration > 1.5:
            
            negligence_command_dict = {
                "Lead": NDECommand(command=Command.TRAJECTORY, duration=2.0, acceleration=ff_acceleration)
            }
            negligence_command_dict["Lead"].future_trajectory = nde_utils.predict_future_trajectory(veh_id, obs_dict, negligence_command_dict["Lead"], self.vehicle.simulator.sumo_net, time_horizon_step=20, time_resolution=0.1)
        # highlight the vehicle with green
        traci.vehicle.highlight(obs_dict["ego"]["veh_id"], (0, 255, 0, 255), 0.5, duration=0.3)

    @staticmethod
    def get_ff_acceleration(obs_dict):
        ff_speed = min(traci.vehicle.getFollowSpeed(obs_dict["ego"]["veh_id"], obs_dict["ego"]["velocity"], 3000, obs_dict["ego"]["velocity"], 7.06), traci.vehicle.getAllowedSpeed(obs_dict["ego"]["veh_id"]))
        return (ff_speed - obs_dict["ego"]["velocity"]) / traci.simulation.getDeltaT()
    
    def lane_change_negligence(self, obs_dict):
        negligence_command_dict = {}

        left_lc_state = traci.vehicle.getLaneChangeStatePretty(obs_dict["ego"]["veh_id"], 1)[1]
        right_lc_state = traci.vehicle.getLaneChangeStatePretty(obs_dict["ego"]["veh_id"], -1)[1]

        if "blocked by left follower" in left_lc_state and "blocked by left leader" not in left_lc_state: # blocked only by left follower
            negligence_command_dict["LeftFoll"] = NDECommand(command=Command.LEFT, duration=1.0)
            # highlight the vehicle with red
            traci.vehicle.highlight(obs_dict["ego"]["veh_id"], (255, 0, 0, 255), 0.5, duration=0.3)
        if "blocked by right follower" in right_lc_state and "blocked by right leader" not in right_lc_state: # blocked only by right follower
            negligence_command_dict["RightFoll"] = NDECommand(command=Command.RIGHT, duration=1.0)
            # highlight the vehicle with red
            traci.vehicle.highlight(obs_dict["ego"]["veh_id"], (255, 0, 0, 255), 0.5, duration=0.3)
        return negligence_command_dict