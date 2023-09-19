from .safetest_nade import SafeTestNADE
from mtlsp.overlay import traci
import mtlsp.utils as utils
import numpy as np

class SafeTestNADEWithAV(SafeTestNADE):

    def __init__(self, vehicle_factory, info_extractor):
        super().__init__(vehicle_factory, info_extractor)
        self.warmup_time = 15*60 # 15 minutes
        self.run_time = 5*60 # 5 minutes

    def on_start(self, ctx):
        super().on_start(ctx)
        self.add_vehicle(veh_id="CAV", route="cav_route", lane="best", lane_id="EG_1_3_1_1", position=0, speed=0)
        traci.vehicle.setColor("CAV", (255, 0, 0, 255))

    def ITE_decision(self, control_command_dict, control_info_dict):
        # track CAV at each timestep using traci GUI track
        # if "CAV" in control_command_dict:
        #     traci.gui.trackVehicle(viewID='View #0', vehID="CAV")
        ITE_control_command_dict, weight = super().ITE_decision(control_command_dict, control_info_dict)
        if "CAV" in control_command_dict:
            ITE_control_command_dict["CAV"] = control_command_dict["CAV"]
        return ITE_control_command_dict, weight
    
    def get_d2rl_obs(self, bv_veh_id):
        CAV_position = list(traci.vehicle.getPosition("CAV"))
        CAV_position_lb = [-5, -5]
        CAV_position_ub = [225, 400]
        
        BV_position = traci.vehicle.getPosition(bv_veh_id)
        BV_cav_relative_position = [BV_position[0] - CAV_position[0], BV_position[1] - CAV_position[1]]

        CAV_speed = traci.vehicle.getSpeed("CAV")
        CAV_speed_lb = 0
        CAV_speed_ub = 20
        
        BV_speed = traci.vehicle.getSpeed(bv_veh_id)
        BV_cav_relative_speed = BV_speed - CAV_speed

        CAV_heading = traci.vehicle.getAngle("CAV")
        BV_heading = traci.vehicle.getAngle(bv_veh_id)

        BV_criticality_value = np.log10(self.importance_sampling_weight)
        bv_criticality_value_lb = -16
        bv_criticality_value_ub = 0

        vehicle_info_lb, vehicle_info_ub = [-20, -20, -10], [20, 20, 10]
        vehicle_info_list = [BV_cav_relative_position[0], BV_cav_relative_position[1], BV_cav_relative_speed]

        lb_array = np.array(CAV_position_lb + [CAV_speed_lb] + [bv_criticality_value_lb] + vehicle_info_lb)
        ub_array = np.array(CAV_position_ub + [CAV_speed_ub] + [bv_criticality_value_ub] + vehicle_info_ub)

        total_obs_for_DRL_ori = np.array(CAV_position + [CAV_speed] + [BV_criticality_value] + vehicle_info_list)
        total_obs_for_DRL = 2 * (total_obs_for_DRL_ori - lb_array)/(ub_array - lb_array) - 1 # normalize the observation
        total_obs_for_DRL = np.clip(total_obs_for_DRL, -5, 5) # clip the observation
        return np.float32(np.array(total_obs_for_DRL))

    def get_cav_info(self, current_time, veh_id, IS_prob, criticality_dict, ndd_control_command_dict):
        infos_dict = {}
        infos_dict["current_time"] = current_time
        infos_dict["criticality_negligence"] = criticality_dict[veh_id]["negligence"]
        infos_dict["ndd_negligence_command"] = ndd_control_command_dict[veh_id]["ndd"]["negligence"]["command"]
        infos_dict["d2rl_obs"] = self.get_d2rl_obs(veh_id).tolist()
        infos_dict["IS_prob"] = IS_prob
        return infos_dict

    def ITE_importance_sampling(self, ndd_control_command_dict, criticality_dict):
        """Importance sampling for NADE.

        Args:
            ndd_control_command_dict (dict): for each vehicle v_id, ndd_control_command_dict[veh_id] = ndd_control_command, ndd_pdf
            criticality_dict (dict): for each vehicle v_id, criticality_dict[veh_id] = criticality

        Returns:
            weight (float): the importance sampling weight
        """
        weight = 1.0
        # ITE_control_command_dict = {veh_id: ndd_control_command_dict[veh_id]["command"] for veh_id in ndd_control_command_dict}
        ITE_control_command_dict = {veh_id: ndd_control_command_dict[veh_id]["ndd"]["normal"]["command"] for veh_id in ndd_control_command_dict}
        
        default_max_IS_prob = 1e-2 # epsilon = 0.95 importance sampling
        for veh_id in criticality_dict:
            if "negligence" in criticality_dict[veh_id] and criticality_dict[veh_id]["negligence"]:
                sampled_prob = np.random.uniform(0, 1)
                ndd_normal_prob = ndd_control_command_dict[veh_id]["ndd"]["normal"]["prob"]
                ndd_negligence_prob = ndd_control_command_dict[veh_id]["ndd"]["negligence"]["prob"]
                assert ndd_normal_prob + ndd_negligence_prob == 1, "The sum of the probabilities of the normal and negligence control commands should be 1."
                IS_prob = default_max_IS_prob
                infos_dict = self.get_cav_info(utils.get_time(), veh_id, IS_prob, criticality_dict, ndd_control_command_dict)
                self.monitor.update_critical_moment_info(veh_id, infos_dict)                
                if sampled_prob < IS_prob: # select the negligece control command
                    weight *= ndd_negligence_prob / IS_prob
                    ITE_control_command_dict[veh_id] = ndd_control_command_dict[veh_id]["ndd"]["negligence"]["command"]
                else:
                    weight *= ndd_normal_prob / (1 - IS_prob)
                    ITE_control_command_dict[veh_id] = ndd_control_command_dict[veh_id]["ndd"]["normal"]["command"]
        return ITE_control_command_dict, weight

    def get_maneuver_challenge(self, negligence_veh_id, negligence_veh_future, all_normal_veh_future):
        cav_future = {"CAV": all_normal_veh_future["CAV"]} if "CAV" in all_normal_veh_future else None
        return super().get_maneuver_challenge(negligence_veh_id, negligence_veh_future, cav_future)

    def should_continue_simulation(self):
        # stop when collision happens or 300s
        collision_id_list = traci.simulation.getCollidingVehiclesIDList()
        num_colliding_vehicles = self.simulator.get_colliding_vehicle_number()
        vehicle_list = traci.vehicle.getIDList()
        if "CAV" not in vehicle_list:
            self._vehicle_in_env_distance("after")
            self.monitor.export_final_state(None, None, self.collision_log(), "CAV_out_of_env")
            return False
        elif num_colliding_vehicles >= 2 and "CAV" in collision_id_list:
            self._vehicle_in_env_distance("after")
            colliding_vehicles = self.simulator.get_colliding_vehicles()
            veh_1_id = colliding_vehicles[0]
            veh_2_id = colliding_vehicles[1]
            self.monitor.save_observation(veh_1_id, veh_2_id)
            self.monitor.export_final_state(veh_1_id, veh_2_id, self.collision_log(), "collision")
            return False
        elif utils.get_time() >= self.warmup_time + self.run_time:
            self._vehicle_in_env_distance("after")
            self.monitor.export_final_state(None, None, self.collision_log(), "timeout")
            return False
        return True
