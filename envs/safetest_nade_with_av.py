import numpy as np

from .safetest_nade import SafeTestNADE
from terasim.overlay import traci
import terasim.utils as utils
import numpy as np

class SafeTestNADEWithAV(SafeTestNADE):

    def on_start(self, ctx):
        super().on_start(ctx)
        self.add_vehicle(veh_id="CAV", route="r_CAV", lane="best", lane_id="EG_35_1_14_0", position=0, speed=0)
        traci.vehicle.setColor("CAV", (255, 0, 0, 255))

    def ITE_decision(self, control_command_dict, control_info_dict):
        # track CAV at each timestep using traci GUI track
        if "CAV" in control_command_dict and self.simulator.gui_flag:
            traci.gui.trackVehicle(viewID='View #0', vehID="CAV")
        ITE_control_command_dict, weight = super().ITE_decision(control_command_dict, control_info_dict)
        if "CAV" in control_command_dict:
            ITE_control_command_dict["CAV"] = control_command_dict["CAV"]
        return ITE_control_command_dict, weight

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
        
        default_max_IS_prob = 0.01 # epsilon = 0.95 importance sampling
        for veh_id in criticality_dict:
            if "negligence" in criticality_dict[veh_id] and criticality_dict[veh_id]["negligence"]:
                sampled_prob = np.random.uniform(0, 1)
                ndd_normal_prob = ndd_control_command_dict[veh_id]["ndd"]["normal"]["prob"]
                ndd_negligence_prob = ndd_control_command_dict[veh_id]["ndd"]["negligence"]["prob"]
                assert ndd_normal_prob + ndd_negligence_prob == 1, "The sum of the probabilities of the normal and negligence control commands should be 1."
                IS_prob = default_max_IS_prob
                # infos_dict = self.get_cav_info(utils.get_time(), veh_id, IS_prob, criticality_dict, ndd_control_command_dict)
                # self.monitor.update_critical_moment_info(veh_id, infos_dict)                
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
