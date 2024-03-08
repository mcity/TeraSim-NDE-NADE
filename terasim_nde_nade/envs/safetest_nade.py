from terasim_nde_nade.envs.safetest_nde import SafeTestNDE
import sumolib
from terasim.overlay import traci
import terasim.utils as utils
import numpy as np
import math
import os
from terasim.utils import sumo_coordinate_to_center_coordinate, sumo_heading_to_orientation
from terasim_nde_nade.vehicle.nde_vehicle_utils import get_collision_type_and_prob, Command, NDECommand, TrajectoryPoint, predict_future_trajectory, get_vehicle_info
from shapely.geometry import LineString
from terasim_nde_nade.vehicle.nde_vehicle_utils import collision_check, is_intersect

veh_length = 5.0
veh_width = 1.85
circle_r = 1.3
tem_len = math.sqrt(circle_r**2-(veh_width/2)**2)


class Point:
    def __init__(self, position_tuple):
        x,y = position_tuple[0], position_tuple[1]
        self.x = x
        self.y = y
    
    def __str__(self) -> str:
        return f"({self.x}, {self.y})"
    
def check_func(PointList):
    for i in range(len(PointList)-1):
        if (PointList[i].x - PointList[i+1].x)**2 + (PointList[i].y - PointList[i+1].y)**2 > 256:
            return False
    return True

import numpy as np


def cal_dist(p1, p2):
    x1, y1 = p1[0], p1[1]
    x2, y2 = p2[0], p2[1]
    return math.sqrt((x1-x2)**2+(y1-y2)**2)


class SafeTestNADE(SafeTestNDE):

    def on_start(self, ctx):
        self.importance_sampling_weight = 1.0
        self.max_importance_sampling_prob = 5e-2
        self.unavoidable_collision_prob_factor = 1e-2 # the factor to reduce the probability of the anavoidable collision
        self.early_termination_weight_threshold = 1e-5
        return super().on_start(ctx)

    # @profile
    def on_step(self, ctx):
        # traci.simulation.executeMove()
        # self._maintain_all_vehicles(ctx)
        self._vehicle_in_env_distance("after")
        # Make NDE decisions for all vehicles
        control_cmds, veh_ctx_dicts = self.make_decisions(ctx)
        obs_dicts = self.get_observation_dicts()
        # Make ITE decision, includes the modification of NDD distribution according to avoidability
        ITE_control_cmds, veh_ctx_dicts, weight = self.NADE_decision(control_cmds, veh_ctx_dicts, obs_dicts) # enable ITE
        # record the negligence mode
        # self.negligence_record(ITE_control_cmds)
        # self.monitor.update_vehicle_mode(ITE_control_cmds, self.importance_sampling_weight)
        # # monitor the environment
        # self.monitor.add_observation(ITE_control_cmds, obs_dicts)
        # weight = 1.0 # disable ITE
        # self.execute_control_commands(ITE_control_cmds)
        self.importance_sampling_weight *= weight # update the importance sampling weight
        # Simulation stop check
        return self.should_continue_simulation()
    
    def get_observation_dicts(self):
        obs_dicts = {
            vehicle.id: vehicle.observation for vehicle in self.vehicle_list.values()
        }
        return obs_dicts
    
    def get_time_to_collision(self, distance, speed):
        if distance <= 0:
            ttc = 0
        elif speed <= 0:
            ttc = 100
        else:
            ttc = distance / speed
        return ttc
    
    # @profile
    def NADE_decision(self, control_command_dicts, veh_ctx_dicts, obs_dicts):
        """NADE decision here.

        Args:
            control_command_dicts
            obs_dicts
        """
        trajectory_dicts, veh_ctx_dicts = self.predict_future_trajectory_dicts(obs_dicts, veh_ctx_dicts)
        maneuver_challenge_dicts, avoidability_dicts, veh_ctx_dicts = self.get_maneuver_challenge_dicts(trajectory_dicts, obs_dicts, veh_ctx_dicts,)
        self.highlight_critical_vehicles(maneuver_challenge_dicts)
        # criticality_dicts, veh_ctx_dicts = self.get_criticality_dicts(control_command_dicts, maneuver_challenge_dicts, veh_ctx_dicts)
        # ITE_control_command_dicts, veh_ctx_dicts, weight = self.ITE_importance_sampling(control_command_dicts, criticality_dicts, veh_ctx_dicts)
        ITE_control_command_dicts, weight = control_command_dicts, 1.0
        return ITE_control_command_dicts, veh_ctx_dicts, weight
    
    def highlight_critical_vehicles(self, maneuver_challenge_dicts):
        for veh_id in maneuver_challenge_dicts:
            if maneuver_challenge_dicts[veh_id]["maneuver_challenge"]:
                # highlight the vehicle with red
                traci.vehicle.highlight(veh_id, (255, 0, 0, 255), duration=0.3)

    # @profile
    def predict_future_trajectory_dicts(self, obs_dicts, veh_ctx_dicts):
        # predict future trajectories for each vehicle
        sumo_net = self.simulator.sumo_net
        current_time = traci.simulation.getTime()
        trajectory_dicts = {}
        ndd_control_command_dicts = {
            veh_id: veh_ctx_dicts[veh_id]["ndd_command_distribution"] for veh_id in veh_ctx_dicts
        }
        for veh_id in ndd_control_command_dicts:
            obs_dict = obs_dicts[veh_id]
            veh_info = get_vehicle_info(veh_id, obs_dict, sumo_net)
            control_command_dict = ndd_control_command_dicts[veh_id]
            obs_dict = obs_dicts[veh_id]
            trajectory_dict = {modality: predict_future_trajectory(veh_id, obs_dict, control_command_dict[modality], sumo_net, time_horizon_step=6, time_resolution=0.5, current_time=current_time, veh_info=veh_info) for modality in control_command_dict}
            trajectory_dicts[veh_id] = trajectory_dict
            # if len(trajectory_dict) > 1:
            #     print(f"veh_id: {veh_id}, trajectory_dict: {trajectory_dict}")
        return trajectory_dicts, veh_ctx_dicts

    def get_modified_ndd_dict_according_to_avoidability(self, ndd_dict, maneuver_challenge_avoidance_dict):
        modified_ndd_dict = {}
        # record the newly updated ndd dict, which is modified according to the maneuver challenge
        new_ndd_dict = {}
        for veh_id in ndd_dict:
            modified_ndd_dict[veh_id] = ndd_dict[veh_id]
            if "negligence" in ndd_dict[veh_id] and "command" in ndd_dict[veh_id]["negligence"]:
                if ("rearend" in modified_ndd_dict[veh_id]["negligence"]["command"]["info"]["predicted_collision_type"]) or (veh_id in maneuver_challenge_avoidance_dict and maneuver_challenge_avoidance_dict[veh_id]["maneuver_challenge"]):
                    modified_ndd_dict[veh_id]["negligence"]["prob"] = modified_ndd_dict[veh_id]["negligence"]["prob"] * self.unavoidable_collision_prob_factor
                    modified_ndd_dict[veh_id]["negligence"]["command"]["info"]["avoidable"] = False
                    modified_ndd_dict[veh_id]["normal"]["prob"] = 1 - modified_ndd_dict[veh_id]["negligence"]["prob"]
                    new_ndd_dict[veh_id] = modified_ndd_dict[veh_id]
        return modified_ndd_dict, new_ndd_dict
    
    def apply_collision_avoidance(self, neglected_vehicle_list, ITE_control_command_dict):
        avoid_collision_IS_prob = float(os.getenv('AVOID_COLLISION_IS_PROB', 0.2))
        avoid_collision_ndd_prob = 0.99
        weight = 1.0
        timestamp = utils.get_time()
        if len(neglected_vehicle_list) == 0:
            return ITE_control_command_dict, weight
        IS_prob = np.random.uniform(0, 1)
        if IS_prob < avoid_collision_IS_prob: # apply collision aboidance (select NDD)
            weight *= avoid_collision_ndd_prob/avoid_collision_IS_prob
            for veh_id in neglected_vehicle_list:
                print(f"time: {timestamp}, neglected vehicle: {veh_id} avoid collision")
                ITE_control_command_dict[veh_id] = {
                    "lateral": "central",
                    "longitudinal": -7.06,
                    "type": "lon_lat",
                    "mode": "avoid_collision",
                }
                self.monitor.update_comprehensive_info(timestamp, {"avoid_collision" + veh_id: {"avoid_collision_prob": avoid_collision_ndd_prob, "IS_prob": IS_prob, "weight": self.importance_sampling_weight, "onestep_weight": avoid_collision_ndd_prob/avoid_collision_IS_prob}})
        else: # does not apply collision avoidance
            print(f"time: {timestamp}, neglected vehicle: {neglected_vehicle_list} accept collision")
            for veh_id in neglected_vehicle_list:
                ITE_control_command_dict[veh_id]["mode"] = "accept_collision"
                self.monitor.update_comprehensive_info(timestamp, {"accept_collision" + veh_id: {"avoid_collision_prob": avoid_collision_ndd_prob, "IS_prob": IS_prob, "weight": self.importance_sampling_weight, "onestep_weight": (1 - avoid_collision_ndd_prob)/(1 - avoid_collision_IS_prob)}})
            weight *= (1 - avoid_collision_ndd_prob)/(1 - avoid_collision_IS_prob)
        return ITE_control_command_dict, weight

    def get_neglecting_vehicle_id(self, control_command_dict, maneuver_challenge_info):
        neglect_pair_list = []
        for veh_id in control_command_dict:
            control_command = control_command_dict[veh_id]
            if "mode" in control_command and control_command["mode"] == "negligence":
                neglecting_vehicle_id = veh_id
                neglected_vehicle_id = list(maneuver_challenge_info[neglecting_vehicle_id].keys())
                neglect_pair_list.append((neglecting_vehicle_id, neglected_vehicle_id))
        return neglect_pair_list
    
    def ITE_importance_sampling(self, ndd_control_command_dict, criticality_dict):
        """Importance sampling for NADE.

        Args:
            ndd_control_command_dict (dict): for each vehicle v_id, ndd_control_command_dict[veh_id] = ndd_control_command, ndd_pdf
            criticality_dict (dict): for each vehicle v_id, criticality_dict[veh_id] = criticality

        Returns:
            weight (float): the importance sampling weight
        """
        weight = 1.0
        ITE_control_command_dict = {veh_id: ndd_control_command_dict[veh_id]["ndd"]["normal"]["command"] for veh_id in ndd_control_command_dict}
        time = utils.get_time()
        for veh_id in criticality_dict:
            if "negligence" in criticality_dict[veh_id] and criticality_dict[veh_id]["negligence"]:
                sampled_prob = np.random.uniform(0, 1)
                ndd_normal_prob = ndd_control_command_dict[veh_id]["ndd"]["normal"]["prob"]
                ndd_negligence_prob = ndd_control_command_dict[veh_id]["ndd"]["negligence"]["prob"]
                assert ndd_normal_prob + ndd_negligence_prob == 1, "The sum of the probabilities of the normal and negligence control commands should be 1."
                IS_prob = self.get_IS_prob(ndd_control_command_dict, criticality_dict, veh_id)
                if sampled_prob < IS_prob: # select the negligece control command
                    weight *= ndd_negligence_prob / IS_prob
                    ITE_control_command_dict[veh_id] = ndd_control_command_dict[veh_id]["ndd"]["negligence"]["command"]
                    self.monitor.update_comprehensive_info(time, {"negligence" + veh_id : {"negligence_prob": ndd_negligence_prob, "IS_prob": IS_prob, "weight": self.importance_sampling_weight, "avoidable": ndd_control_command_dict[veh_id]["ndd"]["negligence"]["command"]["info"]["avoidable"], "onestep_weight": ndd_negligence_prob / IS_prob, "negligence_info": ndd_control_command_dict[veh_id]["ndd"]["negligence"]["command"]["info"]}})
                else:
                    weight *= ndd_normal_prob / (1 - IS_prob)
                    ITE_control_command_dict[veh_id] = ndd_control_command_dict[veh_id]["ndd"]["normal"]["command"]
        return ITE_control_command_dict, weight
    
    def get_IS_prob(self, ndd_control_command_dict, criticality_dict, veh_id):
        if "negligence" in criticality_dict[veh_id] and criticality_dict[veh_id]["negligence"]:
            IS_magnitude = float(os.getenv('IS_MAGNITUDE_INTERSECTION', 20))
            try:
                predicted_collision_type = ndd_control_command_dict[veh_id]["ndd"]["negligence"]["command"]["info"]["predicted_collision_type"]
                if "roundabout" in predicted_collision_type:
                    IS_magnitude = float(os.getenv('IS_MAGNITUDE_ROUNDABOUT', 20))
                    if not ndd_control_command_dict[veh_id]["ndd"]["negligence"]["command"]["info"]["avoidable"]:
                        IS_magnitude = IS_magnitude * 10
                        # print(f"The predicted collision type is roundabout and unavoidable, the IS magnitude is set to {IS_magnitude}")
                    # print(f"The predicted collision type is roundabout, the IS magnitude is set to {IS_magnitude}")
                elif "highway" in predicted_collision_type:
                    IS_magnitude = float(os.getenv('IS_MAGNITUDE_HIGHWAY', 20))
                    # print(f"The predicted collision type is highway, the IS magnitude is set to {IS_magnitude}")
                else:
                    # print(f"The predicted collision type is intersection, the IS magnitude is set to {IS_magnitude}")
                    pass
            except Exception as e:
                print(f"Error: {e}")
                pass

            return np.clip(criticality_dict[veh_id]["negligence"] * IS_magnitude, 0, self.max_importance_sampling_prob)
        else:
            raise Exception("The vehicle is not in the negligence mode.")

    # @profile
    def get_maneuver_challenge_dicts(self, trajectory_dict, obs_dicts, veh_ctx_dicts):
        """Get the maneuver challenge for each vehicle when it is in the negligence mode while other vehicles are in the normal mode.

        Args:
            trajectory_dict (dict): trajectory_dict[veh_id] = {"normal": normal_future_trajectory, "negligence": negligence_future_trajectory}

        Returns:
            maneuver_challenge_dict (dict): maneuver_challenge_dict[veh_id] = (num_affected_vehicles, affected_vehicles)
        """
        normal_future_trajectory_dict = {
            veh_id: trajectory_dict[veh_id].get("normal", None) for veh_id in trajectory_dict
        }
        negligence_future_trajectory_dict = {
            veh_id: trajectory_dict[veh_id].get("negligence", None) for veh_id in trajectory_dict
        }
        avoidance_future_trajectory_dict = {
            veh_id: trajectory_dict[veh_id].get("avoid_collision", None) for veh_id in trajectory_dict
        }
        # get the maneuver challenge for each vehicle, check if the negligence future will collide with other vehicles' normal future
        maneuver_challenge_dicts = {
            veh_id: self.get_maneuver_challenge(veh_id, negligence_future_trajectory_dict[veh_id], normal_future_trajectory_dict, obs_dicts) for veh_id in trajectory_dict
        }
        # get the maneuver challenge for each vehicle, check if the negligence future will collide with other vehicles' avoidance future
        # maneuver_challenge_avoidance_dict = {
        #     veh_id: self.get_maneuver_challenge(veh_id, negligence_future_trajectory_dict[veh_id], avoidance_future_trajectory_dict, highlight_flag=False) for veh_id in trajectory_dict
        # }
        collision_avoidability_dicts = {}
        # for veh_id in maneuver_challenge_dicts:
        #     if veh_id in maneuver_challenge_avoidance_dict and maneuver_challenge_avoidance_dict[veh_id]["maneuver_challenge"]:
        #         collision_avoidability_dicts[veh_id] = "unavoidable" # the vehicle will be considered as unavoidable collision
        #     elif veh_id in maneuver_challenge_dicts and maneuver_challenge_dicts[veh_id]["maneuver_challenge"]:
        #         collision_avoidability_dicts[veh_id] = "avoidable" # the vehicle will be considered as avoidable collision
        # veh_ctx_dicts = {veh_id: veh_ctx_dicts[veh_id].update({"collision_avoidability": collision_avoidability_dicts[veh_id], "maneuver_challenge": maneuver_challenge_dicts[veh_id]}) for veh_id in veh_ctx_dicts}
        return maneuver_challenge_dicts, collision_avoidability_dicts, veh_ctx_dicts

    def get_maneuver_challenge(self, negligence_veh_id, negligence_veh_future, all_normal_veh_future, obs_dicts, highlight_flag=True):
        """Get the maneuver challenge for the negligence vehicle.

        Args:
            negligence_veh_id (str): the id of the negligence vehicle
            negligence_veh_future (list): the future trajectory of the negligence vehicle
            all_normal_veh_future (dict): all_normal_veh _future[veh_id] = future trajectory of the normal vehicle

        Returns:
            num_affected_vehicles (int): the number of vehicles that will be affected by the negligence vehicle
            maneuver_challenge_info (dict): maneuver_challenge_info[veh_id] = 1 if the negligence vehicle will affect the normal vehicle
        """
        # see if the one negligence future will intersect with other normal futures
        maneuver_challenge_info = {}
        if negligence_veh_future is not None and all_normal_veh_future is not None:
            for veh_id in all_normal_veh_future:
                if veh_id == negligence_veh_id:
                    continue
                if all_normal_veh_future[veh_id] is None:
                    print(f"veh_id: {veh_id}, all_normal_veh_future[veh_id]: {all_normal_veh_future[veh_id]}")
                link_intersection_flag = is_link_intersect(obs_dicts[negligence_veh_id], obs_dicts[veh_id])
                if not link_intersection_flag:
                    continue # if the next link of the two vehicles are not intersected, then the two vehicles will not collide
                collision_flag = is_intersect(negligence_veh_future, all_normal_veh_future[veh_id], veh_length, tem_len, circle_r)
                if collision_flag:
                    maneuver_challenge_info[veh_id] = 1
        return {"maneuver_challenge": int(len(maneuver_challenge_info) > 0), "info": maneuver_challenge_info} # the first element is the number of vehicles that will be affected by the negligence vehicle
    
    def get_criticality_dicts(self, control_command_dicts, maneuver_challenge_dicts, veh_ctx_dicts):
        for veh_id in control_command_dicts:
            control_command = control_command_dicts[veh_id]
            if control_command["command"] == "negligence":
                criticality_dicts[veh_id] = self.get_criticality_dict(control_command, maneuver_challenge_dicts[veh_id], veh_ctx_dicts[veh_id])

    def get_criticality_dict(self, ndd_ndd_command_dict, maneuver_challenge_dict, veh_ctx_dict):
        """Get the criticality of the negligence vehicle.

        Args:
            ndd_dict (dict): the ndd pdf of the given BV
            maneuver_challenge_dict (dict): the information of affected vehicles by the negligence vehicle

        Returns:
            criticality_dict (dict): the criticality of the negligence vehicle
        """
        # assert set(ndd_dict.keys()) == set(maneuver_challenge_dict.keys()), "The keys of ndd_dict and maneuver_challenge_dict should be the same."
        criticality_dicts = {}
        for modality in ndd_ndd_command_dict:
            criticality_dicts[modality] = ndd_ndd_command_dict[modality].prob * maneuver_challenge_dict[modality]
        return criticality_dicts, veh_ctx_dict

def is_link_intersect(veh1_obs, veh2_obs):

    veh1_next_lane_id_set = set(veh1_obs["ego"]["upcoming_lanes"])
    veh2_next_lane_id_set = set(veh2_obs["ego"]["upcoming_lanes"])

    if veh1_next_lane_id_set.intersection(veh2_next_lane_id_set):
        return True

    veh1_foe_lane_id_set = veh1_obs["ego"]['upcoming_foe_lane_id_set']
    veh2_foe_lane_id_set = veh2_obs["ego"]['upcoming_foe_lane_id_set']

    # if the next lane of the two vehicles are intersected
    if veh1_foe_lane_id_set.intersection(veh2_next_lane_id_set) or veh2_foe_lane_id_set.intersection(veh1_next_lane_id_set):
        return True
    return False

def get_next_lane_id_set_from_next_links(next_links):
    if len(next_links) == 0:
        return None
    next_lane_id = next_links[0][0]
    via_lane_id = next_links[0][4]
    return set([next_lane_id, via_lane_id])


def get_longitudinal(control_command):
    """Get the longitudinal control command

    Args:
        control_command (dict): control command

    Returns:
        longitudinal (float): longitudinal control command
    """
    if control_command['longitudinal'] == "SUMO":
        return 0
    return control_command['longitudinal']