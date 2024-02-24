from terasim_nde_nade.envs.safetest_nade import SafeTestNADE
from terasim.overlay import traci
import terasim.utils as utils
import numpy as np
from terasim_nde_nade.vehicle.nde_decision_model import IDM_MOBIL_with_negligence

class SafeTestNADEWithAV(SafeTestNADE):

    def on_start(self, ctx):
        # initialize the surrogate model and add AV to env
        self.surrogate_model = IDM_MOBIL_with_negligence(MOBIL_lc_flag=True, stochastic_acc_flag=False)
        super().on_start(ctx)
        self.add_vehicle(veh_id="CAV", route="cav_route", lane="best", lane_id="EG_2_1_1_0", position=0, speed=0)
        traci.vehicle.setColor("CAV", (255, 0, 0, 255))

    def ITE_decision(self, control_command_dict, control_info_dict):
        # track CAV at each timestep using traci GUI track
        if "CAV" in control_command_dict and self.simulator.gui_flag:
            traci.gui.trackVehicle(viewID='View #0', vehID="CAV")
        ITE_control_command_dict, weight, trajectory_dict, maneuver_challenge_dict, criticality_dict = super().ITE_decision(control_command_dict, control_info_dict)
        if "CAV" in control_command_dict:
            ITE_control_command_dict["CAV"] = control_command_dict["CAV"]
        return ITE_control_command_dict, weight, trajectory_dict, maneuver_challenge_dict, criticality_dict

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
        for veh_id in criticality_dict:
            if "negligence" in criticality_dict[veh_id] and criticality_dict[veh_id]["negligence"]:
                sampled_prob = np.random.uniform(0, 1)
                ndd_normal_prob = ndd_control_command_dict[veh_id]["ndd"]["normal"]["prob"]
                ndd_negligence_prob = ndd_control_command_dict[veh_id]["ndd"]["negligence"]["prob"]
                assert ndd_normal_prob + ndd_negligence_prob == 1, "The sum of the probabilities of the normal and negligence control commands should be 1."
                IS_prob = self.get_IS_prob(criticality_dict, veh_id)          
                if sampled_prob < IS_prob: # select the negligece control command
                    weight *= ndd_negligence_prob / IS_prob
                    ITE_control_command_dict[veh_id] = ndd_control_command_dict[veh_id]["ndd"]["negligence"]["command"]
                else:
                    weight *= ndd_normal_prob / (1 - IS_prob)
                    ITE_control_command_dict[veh_id] = ndd_control_command_dict[veh_id]["ndd"]["normal"]["command"]
        return ITE_control_command_dict, weight
    
    def get_IS_prob(self, criticality_dict, veh_id):
        return self.max_importance_sampling_prob

    def predict_future_trajectory_dict(self, veh_id, time_horizon, time_resolution, ndd_decision_dict = None):
        trajectory_dict, veh_info, duration_list = super().predict_future_trajectory_dict(veh_id, time_horizon, time_resolution, ndd_decision_dict)
        if veh_id != "CAV":
            return trajectory_dict, veh_info, duration_list
        else:
            # get CAV future control commmand, including the normal and negligence control commands
            cav_obs_dict = self.vehicle_list[veh_id].observation
            av_control_command, av_control_info, negligence_info = self.surrogate_model.derive_control_command_from_observation_detailed(cav_obs_dict)
            # # get CAV future trajectory using the normal control command
            trajectory_dict_normal, veh_info_normal, duration_list_normal = super().predict_future_trajectory_dict(veh_id, time_horizon, time_resolution, ndd_decision_dict)
            for neg_mode in negligence_info:
                control_command = negligence_info[neg_mode]
                trajectory_dict[neg_mode] = self.predict_future_position_heading(veh_info, neg_mode, control_command["command"], duration_list)
            return trajectory_dict, veh_info, duration_list

    def get_maneuver_challenge(self, negligence_veh_id, negligence_veh_future, all_normal_veh_future, highlight_flag=False):
        cav_future = {"CAV": all_normal_veh_future["CAV"]} if "CAV" in all_normal_veh_future else None
        return super().get_maneuver_challenge(negligence_veh_id, negligence_veh_future, cav_future, highlight_flag)
    
    def should_continue_simulation(self):
        # stop when collision happens or 300s
        collision_id_list = traci.simulation.getCollidingVehiclesIDList()
        num_colliding_vehicles = self.simulator.get_colliding_vehicle_number()
        vehicle_list = traci.vehicle.getIDList()
        if "CAV" not in vehicle_list:
            self._vehicle_in_env_distance("after")
            self.monitor.export_final_state(None, None, self.final_state_log(), "CAV_out_of_env")
            return False
        elif num_colliding_vehicles >= 2 and "CAV" in collision_id_list:
            self._vehicle_in_env_distance("after")
            colliding_vehicles = self.simulator.get_colliding_vehicles()
            veh_1_id = colliding_vehicles[0]
            veh_2_id = colliding_vehicles[1]
            self.monitor.save_observation(veh_1_id, veh_2_id)
            self.monitor.export_final_state(veh_1_id, veh_2_id, self.final_state_log(), "collision")
            return False
        elif utils.get_time() >= self.warmup_time + self.run_time:
            self._vehicle_in_env_distance("after")
            self.monitor.export_final_state(None, None, self.final_state_log(), "timeout")
            return False
        return True
