import terasim.utils as utils
import json
import os
from collections import defaultdict
# from vehicle.vehicle_utils import get_collision_type, get_location

from terasim.overlay import traci
import sumolib

class EnvMonitor:
    def __init__(self, log_dir, exp_id):
        self.observations = {}
        self.num_step = 100
        self.log_dir = log_dir
        final_state_dir = os.path.join(log_dir, "final_state")
        maneuver_challenges_dir = os.path.join(log_dir, "maneuver_challenges")
        if not os.path.exists(final_state_dir):
            os.makedirs(final_state_dir)
        if not os.path.exists(maneuver_challenges_dir):
            os.makedirs(maneuver_challenges_dir)
        self.exp_id = exp_id
        self.total_distance = {"before": {}, "after": {}}
        self.negligence_mode = defaultdict(dict)
        self.num_maneuver_challenges = 0
        self.car_with_maneuver_challenges = defaultdict(set)
        self.init_cav_infos()

    def init_cav_infos(self):
        self.critical_moment_infos = {
            "criticality_step_info": {},
            "ndd_step_info": {},
            "drl_obs_step_info": {},
            "drl_epsilon_step_info": {},
            "real_epsilon_step_info": {},
        }
    
    def bind_env(self, env):
        self.env = env
        self.env.monitor = self
    
    def add_observation(self, control_cmds, obs_dicts):
        current_time = utils.get_time()
        current_observation = self._record(control_cmds, obs_dicts)
        self.update_infos(current_time, current_observation)
        self._lru_collection()

    def update_infos(self, timestamp, veh_observations):
        self.observations.update({timestamp: veh_observations})
        
    def save_observation(self, veh1_id, veh2_id):
        collision_obs = self._fetch_observations(veh1_id, veh2_id)
        with open(os.path.join(f"{self.log_dir}/{self.exp_id}", "monitor.json"), 'w') as f:
            json.dump(collision_obs, f, indent=4, sort_keys=True)
            
    def _fetch_observations(self, veh1_id, veh2_id):
        # only fetch the observations of the last 20 steps related to the collision
        observations = {}
        for timestamp, veh_observations in self.observations.items():
            if veh1_id in veh_observations and veh2_id in veh_observations:
                observations.update({
                    timestamp: {
                        veh1_id: veh_observations[veh1_id],
                        veh2_id: veh_observations[veh2_id]
                    }
                })
        return observations
            
    def _lru_collection(self):
        if len(self.observations) > self.num_step:
            self.observations.pop(min(self.observations.keys()))

    def update_distance(self, distance_dict, mode):
        self.total_distance[mode].update(distance_dict)

    def remove_vehicle_records(self, veh_id_list):
        for veh_id in veh_id_list:
            # self.vehicle_to_route.pop(veh_id)
            self.negligence_mode.pop(veh_id, "exit")
        
    def _record(self, control_cmds, obs_dicts=None):
        current_observation = {}
        current_all_veh_obs = self._veh_obs() if obs_dicts is None else obs_dicts
        for veh_id, control_cmd in control_cmds.items():
            if True:
                current_veh_obs = {
                    "obs": current_all_veh_obs[veh_id],
                    "cmd": control_cmd
                }
                current_observation.update({veh_id: current_veh_obs})
        return current_observation
        
    def _veh_obs(self):
        veh_obs = {veh.id: veh.observation for veh in self.env.vehicle_list}
        filtered_obs = {}
        for veh_id, veh_observation in veh_obs.items():
            filtered_obs.update({veh_id: self._filter_obs(veh_id, veh_observation)})
        return filtered_obs
    
    def _filter_obs(self, veh_id, veh_observation):
        veh_observation = veh_observation["local"][0]
        observation = {
            "Ego": veh_observation["Ego"],
            "speed_mode": utils.get_vehicle_speedmode(veh_id),
            "lane_change_mode": utils.get_vehicle_lanechangemode(veh_id),
            "tfl": utils.get_next_traffic_light(veh_id),
        } # "Ego" is the ego vehicle
        for key in veh_observation.keys():
            if key != "Ego" and veh_observation[key] is not None:
                observation.update({
                    key: veh_observation[key]
                })
        return observation
    
    def update_negligence_mode(self, control_cmds):
        for veh_id, control_cmd in control_cmds.items():
            if control_cmd and "mode" in control_cmd and control_cmd["mode"] == "negligence":
                if len(self.car_with_maneuver_challenges[veh_id]) == 0:
                    print("NegligenceError", veh_id, "", "", utils.get_time(), sep='\t')
                neg_mode = control_cmd["mode"]
                obs_dict = self.env.vehicle_list[veh_id].observation
                self.negligence_mode.update({
                    veh_id: {
                        "mode": neg_mode,
                        "info": control_cmd["info"] if "info" in control_cmd else None,
                        "time": utils.get_time(),
                        "lead_veh": obs_dict["local"].data["Lead"]["veh_id"] if obs_dict["local"].data["Lead"] is not None else None,
                    }
                })
    
    def add_maneuver_challenges(self, maneuver_challenge_dict, time):
        for veh_id, maneuver_challenge in maneuver_challenge_dict.items():
            if maneuver_challenge.get("negligence", 0) == 1:
                self.num_maneuver_challenges += 1
                self.car_with_maneuver_challenges[veh_id].add(time)

    def load_to_json(self, suffix, infos, mode):
        core_id = "_".join(self.exp_id.split("_")[0:-1])
        file_path = os.path.join(f"{self.log_dir}/{mode}", f"{core_id}_{suffix}")
        file_copy_path = os.path.join(f"{self.log_dir}/{mode}", f"{core_id}_copy.json")

        try:
            with open(file_path, "r") as f:
                data = json.load(f)
        except:
            data = {}

        with open(file_copy_path, "w") as f:
            data.update({self.exp_id: infos})
            json.dump(data, f, indent=4)
        os.replace(file_copy_path, file_path)

        with open(os.path.join(f"{self.log_dir}/{self.exp_id}", suffix), 'w') as f:
            json.dump(infos, f, indent=4, sort_keys=True)

    def export_final_state(self, veh_1_id, veh_2_id, final_state_log, end_reason):
        neg_mode, neg_time, neg_car, neg_info = None, -1.0, None, None
        if veh_1_id is not None and veh_2_id is not None:
            veh_1_negligence_mode = self.negligence_mode[veh_1_id]
            veh_2_negligence_mode = self.negligence_mode[veh_2_id]

            veh_1_mode = veh_1_negligence_mode.get("mode", None)
            veh_2_mode = veh_2_negligence_mode.get("mode", None)

            veh_1_neg_info = veh_1_negligence_mode.get("info", None)
            veh_2_neg_info = veh_2_negligence_mode.get("info", None)

            veh_1_neg_time = veh_1_negligence_mode.get("time", -1.0)
            veh_2_neg_time = veh_2_negligence_mode.get("time", -1.0)

            if veh_1_neg_time > veh_2_neg_time:
                neg_mode = veh_1_mode
                neg_time = veh_1_neg_time
                neg_info = veh_1_neg_info
                neg_car = veh_1_id
            else:
                neg_mode = veh_2_mode
                neg_time = veh_2_neg_time
                neg_info = veh_2_neg_info
                neg_car = veh_2_id

        total_distance = 0
        for veh_id in self.total_distance["after"].keys():
            veh_dist_before = self.total_distance["before"].get(veh_id, 0.0)
            veh_dist_after = self.total_distance["after"].get(veh_id, 0.0)
            if veh_dist_after > veh_dist_before:
                total_distance += veh_dist_after - veh_dist_before

        bv_22_total_distance = 0
        for veh_id in self.total_distance["after"].keys():
            if veh_id.startswith("BV_22"):
                veh_dist_before = self.total_distance["before"].get(veh_id, 0.0)
                veh_dist_after = self.total_distance["after"].get(veh_id, 0.0)
                if veh_dist_after > veh_dist_before:
                    bv_22_total_distance += veh_dist_after - veh_dist_before

        experiments_infos = {
            "veh_1_id": veh_1_id,
            "veh_2_id": veh_2_id,
            "end_time": utils.get_time(),
            "negligence_mode": neg_mode,
            "negligence_info": neg_info,
            "negligence_time": neg_time,
            "negligence_car": neg_car,
            "distance": total_distance,
            "bv_22_distance": bv_22_total_distance,
            "end_reason": end_reason,
            "num_maneuver_challenges": self.num_maneuver_challenges,
            "lane_id": traci.vehicle.getLaneID(veh_1_id) if veh_1_id is not None else None,
            "veh_2_lane_id": traci.vehicle.getLaneID(veh_2_id) if veh_2_id is not None else None,
        }
        experiments_infos.update(final_state_log)

        self.load_to_json("final_state.json", experiments_infos, "final_state")
        for veh_id, time_set in self.car_with_maneuver_challenges.items():
            self.car_with_maneuver_challenges[veh_id] = list(time_set)        
        self.load_to_json("maneuver_challenges.json", self.car_with_maneuver_challenges, "maneuver_challenges")
        # self.load_to_json("critical_moment_infos.json", self.critical_moment_infos, "critical_moment_infos")

    def update_critical_moment_info(self, infos_dict):
        # fetch the information from infos_dict
        current_time = infos_dict["current_time"]
        criticality_negligence = infos_dict["criticality_negligence"]
        ndd_negligence_command = infos_dict["ndd_negligence_command"]
        d2rl_obs = infos_dict["d2rl_obs"]
        IS_prob = infos_dict["IS_prob"]
        # update critical information of CAV with the current time
        self.critical_moment_infos["criticality_step_info"].update(current_time, criticality_negligence)
        self.critical_moment_infos["ndd_step_info"].update(current_time, ndd_negligence_command)
        self.critical_moment_infos["drl_obs_step_info"].update(current_time, d2rl_obs)
        self.critical_moment_infos["drl_epsilon_step_info"].update(current_time, 1 - IS_prob)
        self.critical_moment_infos["real_epsilon_step_info"].update(current_time, 1 - IS_prob)