from terasim.vehicle.decision_models.idm_model import IDMModel
import terasim.utils as utils
import numpy as np
import logging
from terasim.overlay import traci

import os
import json

from terasim_nde_ite.vehicle.vehicle_utils import get_collision_type_and_prob, get_location

class IDM_MOBIL_with_negligence(IDMModel):
    
    def __init__(self, MOBIL_lc_flag=True, stochastic_acc_flag=False, lane_config=None):
        if lane_config is None:
            raise ValueError("lane_config is None")
        self.lane_config = lane_config
        super().__init__(MOBIL_lc_flag, stochastic_acc_flag)

    def install(self):
        super().install()
        utils.set_vehicle_speedmode(self.vehicle.id)
        utils.set_vehicle_lanechangemode(self.vehicle.id)
        traci.vehicle.setDecel(self.vehicle.id, 4)
        self.vehicle.simulator.set_vehicle_emegency_deceleration(self.vehicle.id, 4)
    
    def get_negligence_prob(self, obs_dict, negligence_mode):
        # see here for lane to ["roundabout", "freeway", "intersection"]        
        veh_road_id = obs_dict["local"].data["Ego"]["road_id"]
        location = get_location(veh_road_id, self.lane_config)
        neg_veh_road_id, neg_location = None, None
        if obs_dict["local"].data.get(negligence_mode, None) is not None:
            neg_veh_road_id = obs_dict["local"].data[negligence_mode].get("road_id", None)
            neg_location = get_location(neg_veh_road_id, self.lane_config)
        collision_prob, collision_type = get_collision_type_and_prob(obs_dict, negligence_mode, location, neg_location)
        return collision_prob, collision_type
    
    def change_IDM_MOBIL_parameters_from_location(self, obs_dict):
        vehicle_location = get_location(obs_dict["local"].data["Ego"]["road_id"], self.lane_config)
        if "freeway" in vehicle_location: # highway/freeway scenario
            IDM_parameters = {
                "TIME_WANTED": 1.72,
                "DISTANCE_WANTED": 5.92,
                "COMFORT_ACC_MAX": 5.95,
                "COMFORT_ACC_MIN": -5.96,
                "DESIRED_VELOCITY": 28.31,
            }
        elif "intersection" in vehicle_location or "roundabout" in vehicle_location: # urban scenario
            IDM_parameters = {
                "TIME_WANTED": 1.17,
                "DISTANCE_WANTED": 3.28,
                "COMFORT_ACC_MAX": 1.84,
                "COMFORT_ACC_MIN": -1.29,
                "DESIRED_VELOCITY": 10,
            }
        else:
            raise ValueError(f"location {vehicle_location} not supported")
        MOBIL_parameters = {
            "POLITENESS": 0.1,
            "LANE_CHANGE_MIN_ACC_GAIN": 0.2,
            "LANE_CHANGE_MAX_BRAKING_IMPOSED": 3.0,
        }
        self.load_parameters(IDM_parameters, MOBIL_parameters)
        return IDM_parameters, MOBIL_parameters, vehicle_location

    def derive_control_command_from_obs_helper(self, obs_dict):
        if "local" not in obs_dict:
            raise ValueError("No local observation")
        control_command, action_dist, mode = self.decision(obs_dict["local"].data)
        return control_command, action_dist, None

    def derive_control_command_from_observation(self, obs_dict):
        commands, control_info, negligence_info = self.derive_control_command_from_observation_detailed(obs_dict)
        return commands, control_info
    
    def derive_control_command_from_observation_detailed(self, obs_dict):
        IDM_parameters, MOBIL_parameters, vehicle_location = self.change_IDM_MOBIL_parameters_from_location(obs_dict)
        negligence_mode_list_dict = {
            "freeway": ["Lead", "LeftFoll", "RightFoll"],
            "intersection": ["Lead", "LeftFoll", "RightFoll"],
            "roundabout": ["Lead", "LeftFoll", "RightFoll"],
        }
        negligence_mode_list = None
        for key in negligence_mode_list_dict.keys():
            if key in vehicle_location:
                negligence_mode_list = negligence_mode_list_dict[key]
                break
        # vehicle basic info
        veh_id = obs_dict['local'].data['Ego']['veh_id']
        veh_speed = utils.get_speed(veh_id)
        veh_distance_from_departure = utils.get_distance(veh_id)
        veh_tfl = utils.get_next_traffic_light(veh_id)
        
        # compute the control command before negligence
        # control_command_before, info = super().derive_control_command_from_observation(obs_dict)
        _, control_command_before_dist, _ = self.derive_control_command_from_obs_helper(obs_dict)
        
        # choose the normal control command based on the probability
        choice_list = list(control_command_before_dist.keys())
        p_list = [control_command_before_dist[key]["prob"] for key in choice_list]
        command_key = np.random.choice(choice_list, p=p_list)
        control_command_before = control_command_before_dist[command_key]["command"]
        
        commands = control_command_before

        # vehicle commands and info with basic settings
        control_info = {}
        control_info['normal'] = {"command": control_command_before, "prob": 1, "location": vehicle_location}
        control_command_after = None

        # compute the control command after negligence
        # avoid negligence when car is generated or at low speed
        mingap = traci.vehicle.getMinGap(veh_id)
        neg_mode = ""
        control_command_and_mode_list = []
        if veh_distance_from_departure >= 10:
            # consider traffic light, leftfoll, rightfoll, lead negligence mode in order
            if veh_speed >= 2:
                control_command_after = self.tfl_negligence(veh_tfl, control_command_before)
            if control_command_after is None:
                for negligence_mode in negligence_mode_list:
                    neg_obs_dict = obs_dict["local"].data[negligence_mode]
                    if neg_obs_dict is not None:
                        if neg_obs_dict["velocity"] < 0.5 and obs_dict['local'].data['Ego']['velocity'] < 0.5:
                            continue
                        if neg_obs_dict["distance"] < -2:
                            continue
                        elif negligence_mode == "Lead" and neg_obs_dict["distance"] > mingap and veh_speed < 2:
                            continue
                    control_command_after = self.car_negligence(obs_dict, negligence_mode, control_command_before)
                    if control_command_after is not None:
                        control_command_and_mode_list.append((control_command_after, negligence_mode))
            else:
                neg_mode = "TFL"

        negligence_info = {}
        # assign the negligence command to control command
        if len(control_command_and_mode_list) > 0:
            control_command_after, neg_mode = control_command_and_mode_list[0]
        if control_command_after is not None:
            # Variable negligence probability setting
            negligence_prob, neg_info = self.get_negligence_prob(obs_dict, neg_mode)
            # assert "negligence" in control_command_after or "tfl_red" in control_command_after
            control_command_after["mode"] = neg_mode
            control_command_after["info"] = neg_info
            control_info['negligence'] = {"command": control_command_after, "prob": negligence_prob, "location": vehicle_location}
            control_info['normal'] = {"command": control_command_before, "prob": 1 - negligence_prob, "location": vehicle_location}
            negligence_info = {neg_mode: {"command": control_command_after, "prob": 0, "location": vehicle_location} for control_command_after, neg_mode in control_command_and_mode_list}
            # select the control command based on the probability
            prob = np.random.uniform(0, 1)
            if prob < negligence_prob:
                commands = control_command_after
        return commands, control_info, negligence_info
    
    def decision(self, observation):
        """Vehicle decides next action based on IDM model.

        Args:
            observation (dict): Observation of the vehicle.

        Returns:
            float: Action index. 0 represents left turn and 1 represents right turn. If the output is larger than 2, it is equal to the longitudinal acceleration plus 6.
        """
        action = None
        mode = None
        action_dist = {}
        ego_vehicle = observation["Ego"]
        front_vehicle = observation["Lead"]
        # Lateral: MOBIL
        mode = "MOBIL"
        left_gain, right_gain = 0, 0
        left_LC_flag, right_LC_flag = False, False 

        if self.MOBIL_lc_flag:

            # see if the vehicle can change lane
            possible_lane_change = []
            possible_lane_change.append(-1) if observation["Ego"]["could_drive_adjacent_lane_right"] else None
            possible_lane_change.append(1) if observation["Ego"]["could_drive_adjacent_lane_left"] else None
            # calculate the gain of changing lane and the lane change flag
            for lane_index in possible_lane_change:
                LC_flag, gain = self.mobil_gain(lane_index, observation)
                if LC_flag and gain:
                    if lane_index < 0: 
                        right_gain, right_LC_flag = np.clip(gain, 0., None), LC_flag
                    elif lane_index > 0: 
                        left_gain, left_LC_flag = np.clip(gain, 0., None), LC_flag
                        
        left_action = {"lateral": "left", "longitudinal": 0, "type": "lon_lat"}
        right_action = {"lateral": "right", "longitudinal": 0, "type": "lon_lat"}
        tmp_acc = 0
        if not self.stochastic_acc_flag:
            tmp_acc = self.IDM_acceleration(ego_vehicle=ego_vehicle, front_vehicle=front_vehicle)
        else:
            tmp_acc = self.stochastic_IDM_acceleration(ego_vehicle=ego_vehicle, front_vehicle=front_vehicle)
        tmp_acc = np.clip(tmp_acc, self.acc_low, self.acc_high)
        central_action = {"lateral": "central" if self.MOBIL_lc_flag else "SUMO", "longitudinal": tmp_acc, "type": "lon_lat"}
        
        action_dist = {
            "left": {"command": left_action, "prob": 0},
            "right": {"command": right_action, "prob": 0},
            "central": {"command": central_action, "prob": 0},
        }
        
        p = 1e-4
        
        if left_LC_flag or right_LC_flag:
            if right_gain > left_gain:
                action = right_action
                assert(right_gain >= self.LANE_CHANGE_MIN_ACC_GAIN)
                action_dist["right"]["prob"] = p
                action_dist["central"]["prob"] = 1 - p
            else:
                action = left_action
                assert(left_gain >= self.LANE_CHANGE_MIN_ACC_GAIN)
                action_dist["left"]["prob"] = p
                action_dist["central"]["prob"] = 1 - p
        # Longitudinal: IDM
        else:
            mode = "IDM"
            action = central_action
            action_dist["central"]["prob"] = 1
        action["type"] = "lon_lat"
        return action, action_dist, mode
    
    def tfl_negligence(self, veh_tfl, control_command_before):
        """Traffic light negligence function

        Args:
            veh_id (str): vehicle id
            veh_tfl (tuple): traffic light info
            control_command_before (dict): control command before negligence
        
        Returns:
            commands (dict): control command
        """
        commands = None
        if len(veh_tfl) == 4 and (veh_tfl[3] == 'r' or veh_tfl[3] == 'y') and veh_tfl[2] < 4:
            commands = {}
            commands.update(control_command_before)
            commands['longitudinal'] = max(4, commands['longitudinal'])
            commands["tfl_red"] = 1
            commands["mode"] = "negligence"
        return commands

    def car_negligence(self, obs_dict, negligence_mode, control_command_before):
        """Car negligence function

        Args:
            obs_dict (dict): observation
            negligence_mode (str): negligence mode (mainly for lead, leftfoll, rightfoll)
            control_command_before (dict): control command before negligence
        
        Returns:
            commands (dict): control command after negligence
        """
        commands = None
        if obs_dict["local"].data[negligence_mode] is not None:
            # prepare the observation for negligence
            tmp = obs_dict["local"].data[negligence_mode]
            obs_dict["local"].data[negligence_mode] = None
            # compute the control command after negligence
            # commands, _ = super().derive_control_command_from_observation(obs_dict)
            commands, _, _ = self.derive_control_command_from_obs_helper(obs_dict)
            commands['car_negligence'] = 1
            commands["mode"] = "negligence"
            # restore the observation
            obs_dict["local"].data[negligence_mode] = tmp
            # judge the significance of negligence
            if not self.has_siginificance(negligence_mode, control_command_before, commands):
                commands = None
        return commands

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
            longitudinal_acc = np.abs(control_command_after['longitudinal'] - control_command_before['longitudinal']) / 10
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
