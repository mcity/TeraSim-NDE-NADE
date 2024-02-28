from terasim.vehicle.decision_models.idm_model import IDMModel
import terasim.utils as utils
import numpy as np
import logging
from terasim.overlay import traci

import os
import json

from terasim_nde_nade.vehicle.nde_vehicle_utils import get_collision_type_and_prob, get_location

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
        
    def derive_control_command_from_obs_helper(self, obs_dict, is_neglect_flag=False):
        if "local" not in obs_dict:
            raise ValueError("No local observation")
        # obs_dict = self.fix_observation(obs_dict)
        control_command, action_dist, mode = self.decision(obs_dict["local"], is_neglect_flag)
        return control_command, action_dist, None

    def get_negligence_modes(self, obs_dict, vehicle_location=None):
        if vehicle_location is None:
            vehicle_location = get_location(obs_dict["ego"]["edge_id"], self.lane_config)

        # car negligence mode list
        car_negligence_mode_set = set(["Lead"])
        if obs_dict["ego"]["could_drive_adjacent_lane_right"]:
            car_negligence_mode_set.add("RightFoll")
        if obs_dict["ego"]["could_drive_adjacent_lane_left"]:
            car_negligence_mode_set.add("LeftFoll")
        # traffic light negligence mode
        tfl_negligence_mode_set = set()
        # roundabout negligence mode
        rdbt_negligence_mode_set = set()
        return negligence_mode_list
    
    def derive_control_command_from_observation(self, obs_dict):
        # change the IDM and MOBIL parameters based on the location
        vehicle_location = get_location(obs_dict["ego"]["edge_id"], self.lane_config)
        self.change_vehicle_type_according_to_location(obs_dict["ego"]["veh_id"], vehicle_location)

        negligence_mode_set = set()
        # negligence mode list in different locations
        # negligence_mode_set = self.get_negligence_modes(obs_dict, vehicle_location)
        left_lc_state = traci.vehicle.getLaneChangeStatePretty(obs_dict["ego"]["veh_id"], 1)[1]
        right_lc_state = traci.vehicle.getLaneChangeStatePretty(obs_dict["ego"]["veh_id"], -1)[1]

        if "blocked by left follower" in left_lc_state and "blocked by left leader" not in left_lc_state: # blocked only by left follower
            negligence_mode_set.add("LeftFoll")
            # highlight the vehicle with red
            traci.vehicle.highlight(obs_dict["ego"]["veh_id"], (255, 0, 0, 255), 0.5, duration=0.3)
        if "blocked by right follower" in right_lc_state and "blocked by right leader" not in right_lc_state: # blocked only by right follower
            negligence_mode_set.add("RightFoll")
            # highlight the vehicle with red
            traci.vehicle.highlight(obs_dict["ego"]["veh_id"], (255, 0, 0, 255), 0.5, duration=0.3)
        
        current_acceleration = obs_dict["ego"]["acceleration"]

        ff_speed = traci.vehicle.getFollowSpeed(obs_dict["ego"]["veh_id"], obs_dict["ego"]["velocity"], 3000, obs_dict["ego"]["velocity"], 7.06) # assume the leaeding vehicle is 1000m away and with the same speed as the ego vehicle
        ff_acceleration = (ff_speed - obs_dict["ego"]["velocity"]) / traci.simulation.getDeltaT()

        cf_acceleration = current_acceleration
        leader_id, leader_distance = None, None
        leader_info = traci.vehicle.getLeader(obs_dict["ego"]["veh_id"], 40)
        if leader_info is not None:
            leader_id, leader_distance = leader_info

        if leader_id is not None:
            cf_speed_with_leading_vehicle = traci.vehicle.getFollowSpeed(obs_dict["ego"]["veh_id"], obs_dict["ego"]["velocity"], leader_distance, traci.vehicle.getSpeed(leader_id), 7.06)
            cf_acceleration = (cf_speed_with_leading_vehicle - obs_dict["ego"]["velocity"]) / traci.simulation.getDeltaT()

        # no leading vehicle or controlled by traffic rules even with leading vehicle
        if leader_id is None or abs(cf_acceleration - current_acceleration) > 0.5:
            if ff_acceleration - current_acceleration > 1:
                # the vehicle is constained by the traffic rules
                # print("traffic rule violation potential!")
                negligence_mode_set.add("traffic_rule")
                # highlight the vehicle with blue
                traci.vehicle.highlight(obs_dict["ego"]["veh_id"], (0, 0, 255, 255), 0.5, duration=0.3)
 
        if leader_id is not None:
            if ff_acceleration - cf_acceleration > 1.5:
                negligence_mode_set.add("Lead")
                # highlight the vehicle with green
                traci.vehicle.highlight(obs_dict["ego"]["veh_id"], (0, 255, 0, 255), 0.5, duration=0.3)

        return None, None
