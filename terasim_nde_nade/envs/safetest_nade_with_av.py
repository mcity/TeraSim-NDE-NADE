from terasim_nde_nade.envs.safetest_nade import SafeTestNADE
from terasim.overlay import traci
import terasim.utils as utils
import numpy as np
from terasim_nde_nade.vehicle.nde_vehicle_utils import (
    NDECommand,
    Command,
    get_collision_type_and_prob,
    is_car_following,
)
from loguru import logger


class SafeTestNADEWithAV(SafeTestNADE):

    def on_start(self, ctx):
        # initialize the surrogate model and add AV to env
        self.max_importance_sampling_prob = 0.01
        super().on_start(ctx)
        if "CAV" not in traci.vehicle.getIDList():
            self.add_vehicle(
                veh_id="CAV",
                route_id="cav_route",
                lane="best",
                lane_id="EG_35_1_14_0",
                position=0,
                speed=0,
                type_id="ISUZU_truck",
                length=5.4,
                width=1.9,
                height=2.9,
                veh_class="truck",
            )
        # set the CAV with white color
        traci.vehicle.setColor("CAV", (255, 255, 255, 255))

        traci.vehicle.subscribeContext(
            "CAV",
            traci.constants.CMD_GET_VEHICLE_VARIABLE,
            50,
            [traci.constants.VAR_POSITION],
        )

    def reroute_vehicle_if_necessary(self, veh_id, veh_ctx_dicts, obs_dicts):
        if veh_id == "CAV":
            logger.debug("CAV will not be rerouted.")
            return False
        super().reroute_vehicle_if_necessary(veh_id, veh_ctx_dicts, obs_dicts)

    def NADE_decision(self, control_command_dicts, veh_ctx_dicts, obs_dicts):
        predicted_CAV_control_command = self.predict_cav_control_command(
            control_command_dicts, veh_ctx_dicts, obs_dicts
        )
        if predicted_CAV_control_command is not None:
            veh_ctx_dicts["CAV"]["ndd_command_distribution"] = {
                "negligence": predicted_CAV_control_command,
                "normal": NDECommand(command_type=Command.DEFAULT, prob=0),
            }
        return super().NADE_decision(control_command_dicts, veh_ctx_dicts, obs_dicts)

    def predict_future_trajectory_dicts(self, obs_dicts, veh_ctx_dicts):
        # only consider the vehicles that are around the CAV (within 50m range)

        neighbor_veh_ids_set = set(
            traci.vehicle.getContextSubscriptionResults("CAV").keys()
        )
        # add "CAV" to the neighbor_veh_ids_set
        neighbor_veh_ids_set.add("CAV")

        filtered_obs_dicts = {
            veh_id: obs_dicts[veh_id]
            for veh_id in obs_dicts
            if veh_id in neighbor_veh_ids_set
        }
        filtered_veh_ctx_dicts = {
            veh_id: veh_ctx_dicts[veh_id]
            for veh_id in veh_ctx_dicts
            if veh_id in neighbor_veh_ids_set
        }
        return super().predict_future_trajectory_dicts(
            filtered_obs_dicts, filtered_veh_ctx_dicts
        )

    def predict_cav_control_command(
        self, control_command_dicts, veh_ctx_dicts, obs_dicts
    ):
        original_cav_speed = obs_dicts["CAV"]["ego"]["velocity"]
        original_cav_acceleration = obs_dicts["CAV"]["ego"]["acceleration"]
        new_cav_speed = traci.vehicle.getSpeedWithoutTraCI("CAV")
        new_cav_acceleration = (
            new_cav_speed - original_cav_speed
        ) / utils.get_step_size()

        original_cav_angle = obs_dicts["CAV"]["ego"]["heading"]
        cav_lane_id = traci.vehicle.getLaneID("CAV")
        cav_lane_position = traci.vehicle.getLanePosition("CAV")
        cav_lane_angle = traci.lane.getAngle(
            laneID=cav_lane_id,
            relativePosition=max(
                cav_lane_position - 0.5 * traci.vehicle.getLength("CAV"), 0
            ),
        )
        CAV_command = None
        # use the difference between the lane change angle adn the original cav angle to predict the control command (LEFT turn or RIGHT turn)
        # the angle is defined as SUmo's angle, the north is 0, the east is 90, the south is 180, the west is 270
        # the angle is in degree
        angle_diff = (cav_lane_angle - original_cav_angle + 180) % 360 - 180

        if angle_diff > 10:
            CAV_command = NDECommand(
                command_type=Command.LEFT,
                prob=1,
                duration=1.0,
                info={"negligence_mode": "LeftFoll"},
            )
        elif angle_diff < -10:
            CAV_command = NDECommand(
                command_type=Command.RIGHT,
                prob=1,
                duration=1.0,
                info={"negligence_mode": "RightFoll"},
            )

        if original_cav_acceleration - new_cav_acceleration > 1.5:
            # predict the cav control command as negligence
            leader_info = traci.vehicle.getLeader("CAV")
            is_car_following_flag = False
            if leader_info is not None:
                is_car_following_flag = is_car_following("CAV", leader_info[0])
            CAV_command = NDECommand(
                command_type=Command.ACCELERATION,
                acceleration=original_cav_acceleration,
                prob=1,
                duration=1.0,
                info={
                    "negligence_mode": "Lead",
                    "is_car_following_flag": is_car_following_flag,
                },
            )

        if CAV_command:
            _, predicted_collision_type = get_collision_type_and_prob(
                obs_dict=obs_dicts["CAV"],
                negligence_command=CAV_command,
            )
            CAV_command.info.update(
                {"predicted_collision_type": predicted_collision_type}
            )
        return CAV_command

    def get_IS_prob(
        self,
        veh_id,
        ndd_control_command_dicts,
        maneuver_challenge_dicts,
        veh_ctx_dicts,
    ):
        return self.max_importance_sampling_prob

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
        collision_id_list = traci.simulation.getCollidingVehiclesIDList()
        if "CAV" not in traci.vehicle.getIDList():
            logger.info("CAV left the simulation, stop the simulation.")
            return False
        elif len(collision_id_list) >= 2 and "CAV" in collision_id_list:
            logger.critical(
                "Collision happens between CAV and other vehicles, stop the simulation."
            )
            return False
        elif utils.get_time() >= self.warmup_time + self.run_time:
            logger.info("Simulation timeout, stop the simulation.")
            return False
        return True
