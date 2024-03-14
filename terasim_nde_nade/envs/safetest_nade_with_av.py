from terasim_nde_nade.envs.safetest_nade import SafeTestNADE
from terasim.overlay import traci
import terasim.utils as utils
import numpy as np
from terasim_nde_nade.vehicle.nde_vehicle_utils import NDECommand, Command


class SafeTestNADEWithAV(SafeTestNADE):

    def on_start(self, ctx):
        # initialize the surrogate model and add AV to env
        super().on_start(ctx)
        self.add_vehicle(
            veh_id="CAV",
            route="cav_route",
            lane="best",
            lane_id="EG_35_1_14_0",
            position=0,
            speed=0,
        )
        # highlight the CAV with yellow circle
        traci.vehicle.highlight("CAV", (255, 255, 0, 255))

    def NADE_decision(self, control_command_dicts, veh_ctx_dicts, obs_dicts):

        # control_command_dicts["CAV"] = self.predict_cav_control_command(
        #     control_command_dicts, veh_ctx_dicts, obs_dicts
        # )
        return super().NADE_decision(control_command_dicts, veh_ctx_dicts, obs_dicts)

    def predict_cav_control_command(
        self, control_command_dicts, veh_ctx_dicts, obs_dicts
    ):
        cav_original_control_command = control_command_dicts["CAV"]
        original_cav_speed = obs_dicts["CAV"]["velocity"]
        original_cav_acceleration = obs_dicts["CAV"]["acceleration"]
        new_cav_speed = traci.vehicle.getSpeedWithoutTraCI("CAV")
        new_cav_acceleration = (
            new_cav_speed - original_cav_speed
        ) / utils.get_step_size()

        original_cav_angle = obs_dicts["CAV"]["heading"]
        cav_lane_id = traci.vehicle.getLaneID("CAV")
        cav_lane_position = traci.vehicle.getLanePosition("CAV")
        cav_lane_angle = traci.lane.getAngle(
            laneID=cav_lane_id, relativePosition=cav_lane_position
        )
        CAV_command = None
        # use the difference between the lane change angle adn the original cav angle to predict the control command (LEFT turn or RIGHT turn)
        # the angle is defined as SUmo's angle, the north is 0, the east is 90, the south is 180, the west is 270
        # the angle is in degree
        angle_diff = (cav_lane_angle - original_cav_angle + 180) % 360 - 180
        if angle_diff > 0:
            CAV_command = NDECommand(command_type=Command.LEFT, prob=1, duration=1.0)
        elif angle_diff < 0:
            CAV_command = NDECommand(command_type=Command.RIGHT, prob=1, duration=1.0)

        if original_cav_acceleration - new_cav_acceleration > 1.5:
            # predict the cav control command as negligence
            CAV_command = NDECommand(
                command_type=Command.ACCELERATION,
                acceleration=original_cav_acceleration,
                prob=1,
                duration=1.0,
            )
        return CAV_command

    # def get_IS_prob(self, criticality_dict, veh_id):
    #     return self.max_importance_sampling_prob

    def get_maneuver_challenge(
        self,
        negligence_veh_id,
        negligence_veh_future,
        all_normal_veh_future,
        obs_dicts,
        veh_ctx_dict,
        record_in_ctx=False,
        highlight_flag=True,
    ):
        if negligence_veh_id != "CAV":
            cav_future = (
                {"CAV": all_normal_veh_future["CAV"]}
                if "CAV" in all_normal_veh_future
                else None
            )
            return super().get_maneuver_challenge(
                negligence_veh_id,
                negligence_veh_future,
                cav_future,
                obs_dicts,
                veh_ctx_dict,
                record_in_ctx,
                highlight_flag,
            )
        else:
            return super().get_maneuver_challenge(
                negligence_veh_id,
                negligence_veh_future,
                all_normal_veh_future,
                obs_dicts,
                veh_ctx_dict,
                record_in_ctx,
                highlight_flag,
            )

    def should_continue_simulation(self):
        # stop when collision happens or 300s
        collision_id_list = traci.simulation.getCollidingVehiclesIDList()
        vehicle_list = traci.vehicle.getIDList()
        if "CAV" not in vehicle_list:
            self._vehicle_in_env_distance("after")
            self.monitor.export_final_state(
                None, None, self.final_state_log(), "CAV_out_of_env"
            )
            return False
        elif len(collision_id_list) >= 2 and "CAV" in collision_id_list:
            self._vehicle_in_env_distance("after")
            colliding_vehicles = self.simulator.get_colliding_vehicles()
            veh_1_id = colliding_vehicles[0]
            veh_2_id = colliding_vehicles[1]
            self.monitor.save_observation(veh_1_id, veh_2_id)
            self.monitor.export_final_state(
                veh_1_id, veh_2_id, self.final_state_log(), "collision"
            )
            return False
        elif utils.get_time() >= self.warmup_time + self.run_time:
            self._vehicle_in_env_distance("after")
            self.monitor.export_final_state(
                None, None, self.final_state_log(), "timeout"
            )
            return False
        return True
