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
from addict import Dict
import copy
from terasim.envs.template import EnvTemplate


class SafeTestNADEWithAV(SafeTestNADE):

    def __init__(self, cache_radius=100, control_radius=50, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cache_radius = cache_radius
        self.control_radius = control_radius

    def cache_vehicle_history(self):
        cav_context = traci.vehicle.getContextSubscriptionResults("CAV")
        if cav_context:
            cached_veh_ids = cav_context.keys()
            for veh_id in cached_veh_ids:
                _ = self.vehicle_list[
                    veh_id
                ].observation  # get the observation of the vehicle to cache it
            # filter the cached_veh_ids by the controlled radius to controlled_veh_ids
            controlled_veh_ids = []
            for veh_id in cav_context:
                if (
                    cav_context[veh_id][traci.constants.VAR_DISTANCE]
                    <= self.control_radius
                ):
                    controlled_veh_ids.append(veh_id)
            return cached_veh_ids, controlled_veh_ids
        else:
            raise ValueError("CAV context is empty")

    def on_start(self, ctx):
        # initialize the surrogate model and add AV to env
        self.max_importance_sampling_prob = 0.01
        super().on_start(ctx)
        self.add_cav()

    def add_cav(self):
        self.add_vehicle(
            veh_id="CAV",
            route_id="cav_route",
            lane="best",
            lane_id="EG_35_1_14_0",
            position=0,
            speed=0,
        )
        # set the CAV with white color
        traci.vehicle.setColor("CAV", (255, 255, 255, 255))

        traci.vehicle.subscribeContext(
            "CAV",
            traci.constants.CMD_GET_VEHICLE_VARIABLE,
            self.cache_radius,
            [traci.constants.VAR_DISTANCE],
        )

    def on_step(self, ctx):
        cached_veh_ids, controlled_veh_ids = self.cache_vehicle_history()
        # ctx["terasim_controlled_vehicle_ids"] = set(controlled_veh_ids)
        return super().on_step(ctx)

    def reroute_vehicle_if_necessary(self, veh_id, veh_ctx_dicts, obs_dicts):
        if veh_id == "CAV":
            logger.debug("CAV will not be rerouted.")
            return False
        super().reroute_vehicle_if_necessary(veh_id, veh_ctx_dicts, obs_dicts)

    def NADE_decision(self, control_command_dicts, veh_ctx_dicts, obs_dicts):
        predicted_CAV_control_command = self.predict_cav_control_command(
            control_command_dicts, veh_ctx_dicts, obs_dicts
        )
        if veh_ctx_dicts["CAV"] is None:
            veh_ctx_dicts["CAV"] = Dict()
        if predicted_CAV_control_command is not None:
            veh_ctx_dicts["CAV"]["ndd_command_distribution"] = Dict(
                {
                    "negligence": predicted_CAV_control_command,
                    "normal": NDECommand(command_type=Command.DEFAULT, prob=0),
                }
            )
        else:
            veh_ctx_dicts["CAV"]["ndd_command_distribution"] = Dict(
                {
                    "normal": NDECommand(command_type=Command.DEFAULT, prob=1),
                }
            )
        CAV_command_cache = copy.deepcopy(control_command_dicts["CAV"])
        control_command_dicts["CAV"] = NDECommand(command_type=Command.DEFAULT, prob=1)
        (
            ITE_control_command_dicts,
            veh_ctx_dicts,
            weight,
            trajectory_dicts,
            maneuver_challenge_dicts,
            criticality_dicts,
        ) = super().NADE_decision(control_command_dicts, veh_ctx_dicts, obs_dicts)
        ITE_control_command_dicts["CAV"] = CAV_command_cache
        return (
            ITE_control_command_dicts,
            veh_ctx_dicts,
            weight,
            trajectory_dicts,
            maneuver_challenge_dicts,
            criticality_dicts,
        )

    def on_step(self, ctx):
        self.distance_info.after.update(self.update_distance())
        self.record.final_time = utils.get_time()  # update the final time at each step
        self.cache_history_tls_data()
        # clear vehicle context dicts
        veh_ctx_dicts = {}
        # Make NDE decisions for all vehicles
        control_cmds, veh_ctx_dicts = EnvTemplate.make_decisions(self, ctx)
        CAV_control_command_cache = (
            copy.deepcopy(control_cmds["CAV"]) if "CAV" in control_cmds else None
        )
        # first_vehicle_veh = list(control_cmds.keys())[0]
        # for veh_id in control_cmds:
        #     history_data = self.vehicle_list[veh_id].sensors["ego"].history_array
        obs_dicts = self.get_observation_dicts()
        # Make ITE decision, includes the modification of NDD distribution according to avoidability
        control_cmds, veh_ctx_dicts, obs_dicts, should_continue_simulation_flag = (
            self.executeMove(ctx, control_cmds, veh_ctx_dicts, obs_dicts)
        )
        if "CAV" in traci.vehicle.getIDList():
            (
                ITE_control_cmds,
                veh_ctx_dicts,
                weight,
                trajectory_dicts,
                maneuver_challenge_dicts,
                _,
            ) = self.NADE_decision(
                control_cmds, veh_ctx_dicts, obs_dicts
            )  # enable ITE
            self.importance_sampling_weight *= weight  # update weight by negligence
            ITE_control_cmds, veh_ctx_dicts, weight = self.apply_collision_avoidance(
                trajectory_dicts, veh_ctx_dicts, ITE_control_cmds
            )
            self.importance_sampling_weight *= (
                weight  # update weight by collision avoidance
            )
            ITE_control_cmds = self.update_control_cmds_from_predicted_trajectory(
                ITE_control_cmds, trajectory_dicts
            )
            if hasattr(self, "nnde_make_decisions"):
                nnde_control_commands, _ = self.nnde_make_decisions(ctx)
                ITE_control_cmds = self.merge_NADE_NeuralNDE_control_commands(
                    ITE_control_cmds, nnde_control_commands
                )
            self.refresh_control_commands_state()
            if "CAV" in ITE_control_cmds and CAV_control_command_cache is not None:
                ITE_control_cmds["CAV"] = CAV_control_command_cache
            self.execute_control_commands(ITE_control_cmds)
            self.record_step_data(veh_ctx_dicts)
        return should_continue_simulation_flag

    def update_control_cmds_from_predicted_trajectory(
        self, ITE_control_cmds, trajectory_dicts
    ):
        # only apply this function to other vehicles except CAV
        if "CAV" not in ITE_control_cmds:
            return super().update_control_cmds_from_predicted_trajectory(
                ITE_control_cmds, trajectory_dicts
            )
        cav_command_cache = ITE_control_cmds.pop("CAV")
        ITE_control_cmds = super().update_control_cmds_from_predicted_trajectory(
            ITE_control_cmds, trajectory_dicts
        )
        ITE_control_cmds["CAV"] = cav_command_cache
        return ITE_control_cmds

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

    def calculate_total_distance(self):
        return traci.vehicle.getDistance("CAV")
        # total_distance = 0
        # veh_id = "CAV"
        # if veh_id not in self.distance_info.before:
        #     total_distance += self.distance_info.after[veh_id]
        # else:
        #     total_distance += (
        #         self.distance_info.after[veh_id] - self.distance_info.before[veh_id]
        #     )
        # return total_distance

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
        buffer=0,
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
                buffer=buffer,
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
                buffer=buffer,
            )

    def should_continue_simulation(self):
        num_colliding_vehicles = self.simulator.get_colliding_vehicle_number()
        self._vehicle_in_env_distance("after")

        if num_colliding_vehicles >= 2:
            colliding_vehicles = self.simulator.get_colliding_vehicles()
            veh_1_id = colliding_vehicles[0]
            veh_2_id = colliding_vehicles[1]
            self.record.update(
                {
                    "veh_1_id": veh_1_id,
                    "veh_1_obs": self.vehicle_list[veh_1_id].observation,
                    "veh_2_id": veh_2_id,
                    "veh_2_obs": self.vehicle_list[veh_2_id].observation,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                    "finish_reason": "collision",
                }
            )
            return False
        elif "CAV" not in traci.vehicle.getIDList():
            logger.info("CAV left the simulation, stop the simulation.")
            self.record.update(
                {
                    "veh_1_id": None,
                    "veh_2_id": None,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                    "finish_reason": "CAV_left",
                }
            )
            return False
        elif utils.get_time() >= self.warmup_time + self.run_time:
            logger.info("Simulation timeout, stop the simulation.")
            self.record.update(
                {
                    "veh_1_id": None,
                    "veh_2_id": None,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                    "finish_reason": "timeout",
                }
            )
            return False
        return True
