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
        veh_road_id = obs_dict["local"]["Ego"]["edge_id"]
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
        # self.change_vehicle_type_according_to_location(obs_dict["ego"]["veh_id"], vehicle_location)

        # negligence mode list in different locations
        # negligence_mode_set = self.get_negligence_modes(obs_dict, vehicle_location)
        # lanechange_state = traci.vehicle.getLaneChangeState(obs_dict["ego"]["veh_id"], -1)
        # print(traci.vehicle.getAcceleration(obs_dict["ego"]["veh_id"]))
        control_command, info = None, None
        return control_command, info
    
    
    def rdbt_fail_to_yield_negligence(self, control_command_nde, veh_info, obs_dict):
        veh_distance_from_departure = veh_info["veh_distance_from_departure"]
        
        rdbt_negligence_command = None
        if veh_distance_from_departure >= 10: # avoid negligence when car is generated
            if self.is_stopping_in_front_of_stop_line_rdbt(veh_info, obs_dict):
                rdbt_negligence_command = {}
                rdbt_negligence_command.update(control_command_nde)
                rdbt_negligence_command['longitudinal'] = max(4, rdbt_negligence_command['longitudinal'])
                rdbt_negligence_command["rdbt"] = 1
                rdbt_negligence_command["mode"] = "negligence"
        return rdbt_negligence_command
    
    def is_stopping_in_front_of_stop_line_rdbt(self, veh_info, obs_dict):
        veh_speed = veh_info["veh_speed"] # check if the vehicle is stopping in front of the stop line
        no_lead_flag = (obs_dict["local"]["Lead"] is None) # vehicle has no leading vehicle
        veh_edge_id = obs_dict["ego"]["edge_id"]
        roundabout_incoming_edge_set = set(["EG_20_1_1", "EG_22_1_1", "EG_9_1_1", "EG_19_1_1", "EG_23_2_1", "EG_10_1_1", "EG_16_1_5"])
        if veh_edge_id in roundabout_incoming_edge_set and no_lead_flag and veh_speed < 0.5:
            # traci set vehicle color to green
            # traci.vehicle.setColor(veh_info["veh_id"], (0, 255, 0, 255))
            return True
        return False
    
    def tfl_negligence(self, control_command_nde, veh_info):
        """Traffic light negligence function

        Args:
            veh_id (str): vehicle id
            veh_tfl (tuple): traffic light info
            control_command_before (dict): control command before negligence
        
        Returns:
            commands (dict): control command
        """
        veh_distance_from_departure = veh_info["veh_distance_from_departure"]
        veh_speed = veh_info["veh_speed"]
        veh_tfl = veh_info["veh_tfl"]
        tfl_negligence_command = None
        if veh_distance_from_departure >= 10 and veh_speed >= 2: # avoid negligence when car is generated or at low speed
            if len(veh_tfl) == 4 and (veh_tfl[3] == 'r' or veh_tfl[3] == 'y') and veh_tfl[2] < 4:
                tfl_negligence_command = {}
                tfl_negligence_command.update(control_command_nde)
                tfl_negligence_command['longitudinal'] = max(4, tfl_negligence_command['longitudinal'])
                tfl_negligence_command["tfl_red"] = 1
                tfl_negligence_command["mode"] = "negligence"
                # set the vehicle color to blue
                # traci.vehicle.setColor(veh_info["veh_id"], (0, 0, 255, 255))
        return tfl_negligence_command

    def car_negligence(self, obs_dict, negligence_mode, control_command_nde, veh_info):
        """Car negligence function

        Args:
            obs_dict (dict): observation
            negligence_mode (str): negligence mode (mainly for lead, leftfoll, rightfoll)
            control_command_before (dict): control command before negligence
        
        Returns:
            commands (dict): control command after negligence
        """
        veh_distance_from_departure = veh_info["veh_distance_from_departure"]
        veh_speed = veh_info["veh_speed"]
        veh_id = veh_info["veh_id"]
        mingap = traci.vehicle.getMinGap(veh_id)
        car_negligence_command = None
        if veh_distance_from_departure >= 10:
            neg_obs_dict = obs_dict["local"][negligence_mode] # neglected vehicle observation
            if neg_obs_dict is not None:
                if neg_obs_dict["velocity"] < 0.5 and obs_dict['local']['Ego']['velocity'] < 0.5: # both vehicles are stopped
                    return car_negligence_command
                if neg_obs_dict["distance"] < -2: # the vehicle is behind the ego vehicle, especially for the cutin scenario
                    return car_negligence_command
                elif negligence_mode == "Lead" and neg_obs_dict["distance"] > mingap and veh_speed < 2:
                    return car_negligence_command

                # prepare the observation for negligence
                tmp = obs_dict["local"][negligence_mode]
                obs_dict["local"][negligence_mode] = None
                car_negligence_command, _, _ = self.derive_control_command_from_obs_helper(obs_dict, is_neglect_flag=True)
                car_negligence_command['car_negligence'] = 1
                car_negligence_command["mode"] = "negligence"
                # restore the observation
                obs_dict["local"][negligence_mode] = tmp
                # judge the significance of negligence
                if not self.has_siginificance(negligence_mode, control_command_nde, car_negligence_command):
                    car_negligence_command = None
        return car_negligence_command

    @staticmethod
    def has_siginificance(negligence_mode, control_command_before, control_command_after):
        """Judge the significance of negligence

        Args:
            negligence_mode (str): negligence mode (mainly for lead, leftfoll, rightfoll)
            control_command_before (dict): control command before negligence
            control_command_after (dict): control command after negligence

        Returns:
            is_significant (bool): whether the negligence is significant
        """
        # judge the movement of the vehicle is same or not
        is_significant = False
        if control_command_after is not None:
            lateral_flag = control_command_after['lateral'] != control_command_before['lateral']
            # calculate the longitudinal acceleration difference with normalization
            longitudinal_acc = np.abs(get_longitudinal(control_command_after) - get_longitudinal(control_command_before)) / 10
            # judge the negligence mode corresponds to the movement of the vehicle when changing lane
            lane_change_flag = (control_command_after['lateral'] == "left" and negligence_mode in ["LeftLead", "LeftFoll"]) \
                or (control_command_after['lateral'] == "right" and negligence_mode in ["RightLead", "RightFoll"])
            # judge the vehicle is not changing lane
            lane_unchange_flag = negligence_mode in ["Lead", "Foll"]
            # vehicle acceleration difference threshold
            speed_flag = longitudinal_acc > 0.15
            # judge the significance of negligence based on the above conditions:
            # 1. lane change and (different lateral movement or significant speed difference)
            # 2. lane unchange and significant speed difference
            is_significant = (lane_change_flag and (lateral_flag or speed_flag)) or (lane_unchange_flag and speed_flag) 
        return is_significant
