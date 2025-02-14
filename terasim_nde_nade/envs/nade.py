import json
import math
import os

import numpy as np
import terasim.utils as utils
from addict import Dict
from loguru import logger
from terasim.envs.template_complete import EnvTemplateComplete
from terasim.overlay import profile, traci
from terasim.params import AgentType

from terasim_nde_nade.envs.nde import NDE
from terasim_nde_nade.utils import (
    CommandType,
    NDECommand,
    get_vehicle_info,
    is_intersect,
    predict_future_trajectory,
)
from terasim_nde_nade.utils.agents.vru import (
    get_vulnerbale_road_user_info,
    predict_future_trajectory_vulnerable_road_user,
)

veh_length = 5.0
veh_width = 1.85
circle_r = 1.3
tem_len = math.sqrt(circle_r**2 - (veh_width / 2) ** 2)
IS_MAGNITUDE_DEFAULT = 20
IS_MAGNITUDE_MULTIPLIER = 40
IS_MAGNITUDE_MAPPING = {
    "roundabout": "IS_MAGNITUDE_ROUNDABOUT",
    "highway": "IS_MAGNITUDE_HIGHWAY",
    "intersection": "IS_MAGNITUDE_INTERSECTION",
}


class Point:
    def __init__(self, position_tuple):
        x, y = position_tuple[0], position_tuple[1]
        self.x = x
        self.y = y

    def __str__(self) -> str:
        return "({}, {})".format(self.x, self.y)


BaseEnv = NDE  # SafeTestNDE


class NADE(BaseEnv):
    def on_start(self, ctx):
        self.importance_sampling_weight = 1.0
        self.max_importance_sampling_prob = 5e-2
        self.unavoidable_collision_prob_factor = (
            3e-4  # the factor to reduce the probability of the anavoidable collision
        )
        self.log = Dict()
        on_start_result = super().on_start(ctx)
        self.distance_info = Dict({"before": self.update_distance(), "after": Dict()})
        self.allow_NADE_IS = True
        self.latest_IS_time = -1
        return on_start_result

    @profile
    def on_step(self, ctx):
        self.distance_info.after.update(self.update_distance())
        self.record.final_time = utils.get_time()  # update the final time at each step
        self.cache_history_tls_data()
        # clear vehicle context dicts
        ctx_dicts = {}
        # Make NDE decisions for all vehicles and vrus
        control_cmds, ctx_dicts = EnvTemplateComplete.make_decisions(self, ctx)
        # first_vehicle_veh = list(control_cmds.keys())[0]
        # for veh_id in control_cmds:
        #     history_data = self.vehicle_list[veh_id].sensors["ego"].history_array
        obs_dicts = self.get_observation_dicts()
        # Make ITE decision, includes the modification of NDD distribution according to avoidability
        (
            control_cmds,
            ctx_dicts,
            obs_dicts,
            should_continue_simulation_flag,
        ) = self.executeMove(ctx, control_cmds, ctx_dicts, obs_dicts)
        # if should_continue_simulation_flag:
        (
            ITE_control_cmds,
            ctx_dicts,
            weight,
            trajectory_dicts,
            maneuver_challenge_dicts,
            _,
        ) = self.NADE_decision(
            control_cmds, ctx_dicts, obs_dicts
        )  # enable ITE
        self.importance_sampling_weight *= weight  # update weight by negligence
        ITE_control_cmds, ctx_dicts, weight = self.apply_collision_avoidance(
            trajectory_dicts, ctx_dicts, ITE_control_cmds
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
        self.execute_control_commands(ITE_control_cmds)

        self.record_step_data(ctx_dicts)
        return should_continue_simulation_flag

    def on_stop(self, ctx) -> bool:
        if self.log_flag:
            self.distance_info.after.update(self.update_distance())
            self.record.weight = self.importance_sampling_weight
            self.record.total_distance = self.calculate_total_distance()
            logger.info(f"total distance: {self.record.total_distance}")
            self.align_record_event_with_collision()
            moniotr_json_path = self.log_dir / "monitor.json"
            with open(moniotr_json_path, "w") as f:
                json.dump(self.record, f, indent=4, default=str)
        return super().on_stop(ctx)

    def calculate_total_distance(self):
        total_distance = 0
        for veh_id in self.distance_info.after:
            if veh_id not in self.distance_info.before:
                total_distance += self.distance_info.after[veh_id]
            else:
                total_distance += (
                    self.distance_info.after[veh_id] - self.distance_info.before[veh_id]
                )
        return total_distance

    # find the corresponding event that lead to the final result (e.g., collisions)
    def align_record_event_with_collision(self):
        if self.record["finish_reason"] == "collision":
            veh_1_id = self.record["veh_1_id"]
            veh_2_id = self.record["veh_2_id"]
            # find if one of veh_1_id or veh_2_id is in the record.event_info negligence_pair_dict
            for timestep in self.record.event_info:
                negligence_pair_dict = self.record.event_info[
                    timestep
                ].negligence_pair_dict
                negligence_related_vehicle_set = set()
                for neglecting_veh_id in negligence_pair_dict:
                    negligence_related_vehicle_set.add(neglecting_veh_id)
                    negligence_related_vehicle_set.update(
                        negligence_pair_dict[neglecting_veh_id]
                    )
                if set([veh_1_id, veh_2_id]).issubset(negligence_related_vehicle_set):
                    self.record.negligence_event_time = timestep
                    self.record.negligence_event_info = self.record.event_info[timestep]

    def merge_NADE_NeuralNDE_control_commands(
        self, NADE_control_commands, NeuralNDE_control_commands
    ):
        # only replace the control commands that command_type is DEFAULT
        for veh_id in NeuralNDE_control_commands:
            if (
                veh_id in NADE_control_commands
                and NADE_control_commands[veh_id].command_type == CommandType.DEFAULT
            ):
                NADE_control_commands[veh_id] = NeuralNDE_control_commands[veh_id]
                NADE_control_commands[veh_id] = NeuralNDE_control_commands[veh_id]
        return NADE_control_commands

    def record_step_data(self, veh_ctx_dicts):
        step_log = Dict()
        for veh_id, veh_ctx_dict in veh_ctx_dicts.items():
            maneuver_challenge = veh_ctx_dict.get("maneuver_challenge", None)
            if maneuver_challenge and maneuver_challenge.get("negligence", None):
                step_log[veh_id]["maneuver_challenge"] = maneuver_challenge

            keys = ["avoidable", "conflict_vehicle_list", "mode"]
            step_log[veh_id].update(
                {key: veh_ctx_dict[key] for key in keys if veh_ctx_dict.get(key)}
            )
            if step_log[veh_id].get("avoidable"):
                step_log[veh_id].pop(
                    "avoidable"
                )  # remove the avoidable key if it is True
        # pop the empty dict
        step_log = {k: v for k, v in step_log.items() if v}
        step_log = {
            "weight": self.importance_sampling_weight,
            "vehicle_log": step_log,
        }
        self.record.step_info[utils.get_time()] = step_log
        return step_log

    def update_distance(self):
        distance_info_dict = Dict()
        for veh_id in traci.vehicle.getIDList():
            distance_info_dict[veh_id] = traci.vehicle.getDistance(veh_id)
        return distance_info_dict

    def record_final_data(self, veh_ctx_dicts):
        # return False
        collide_veh_list = traci.simulation.getCollidingVehiclesIDList()
        if len(collide_veh_list) == 0:
            veh_1_id, veh_2_id = None, None
        elif len(collide_veh_list) >= 2:
            veh_1_id, veh_2_id = collide_veh_list[:2]
        else:
            raise ValueError(
                "The number of colliding vehicles is less than 2 but not 0."
            )
        experiments_infos = {
            "veh_1_id": veh_1_id,
            "veh_2_id": veh_2_id,
            "end_time": utils.get_time(),
            "negligence_mode": neg_mode,
            "negligence_info": neg_info,
            "negligence_time": neg_time,
            "negligence_car": neg_car,
            "avoid_collision_mode": avoid_mode,
            "avoid_collision_info": avoid_info,
            "avoid_collision_time": avoid_time,
            "avoid_collision_car": avoid_car,
            "accept_collision_mode": accept_mode,
            "accept_collision_info": accept_info,
            "accept_collision_time": accept_time,
            "accept_collision_car": accept_car,
            "distance": total_distance,
            "end_reason": end_reason,
            "num_maneuver_challenges": self.num_maneuver_challenges,
            "veh_1_lane_id": (
                traci.vehicle.getLaneID(veh_1_id) if veh_1_id is not None else None
            ),
            "veh_2_lane_id": (
                traci.vehicle.getLaneID(veh_2_id) if veh_2_id is not None else None
            ),
        }

    def get_time_to_collision(self, distance, speed):
        if distance <= 0:
            ttc = 0
        elif speed <= 0:
            ttc = 100
        else:
            ttc = distance / speed
        return ttc

    @profile
    def NADE_decision(self, control_command_dicts, ctx_dicts, obs_dicts):
        """NADE decision here.

        Args:
            control_command_dicts
            obs_dicts
        """
        trajectory_dicts, ctx_dicts = self.predict_future_trajectory_dicts(
            obs_dicts, ctx_dicts
        )  # predict future trajectories for each vehicle (normal/negligence)

        # get maneuver challenge and collision avoidability
        # will mark the conflict vehicles in the veh_ctx_dicts
        maneuver_challenge_dicts, ctx_dicts = self.get_maneuver_challenge_dicts(
            trajectory_dicts,
            obs_dicts,
            ctx_dicts,
        )

        # add collision avoidance command for the neglected vehicles, and predict the future trajectories for the avoidance command using the veh_ctx_dicts
        trajectory_dicts, ctx_dicts = self.add_avoid_accept_collision_command(
            obs_dicts, trajectory_dicts, ctx_dicts
        )

        # update the ndd probability according to collision avoidability
        avoidability_dicts, ctx_dicts = self.get_avoidability_dicts(
            maneuver_challenge_dicts,
            trajectory_dicts,
            obs_dicts,
            ctx_dicts,
        )

        ctx_dicts = self.remove_collision_avoidance_command_using_avoidability(
            obs_dicts, trajectory_dicts, ctx_dicts
        )

        # update the ndd probability according to collision avoidability
        (
            modified_ndd_control_command_dicts,
            ctx_dicts,
        ) = self.modify_ndd_dict_according_to_avoidability(
            maneuver_challenge_dicts, ctx_dicts
        )
        ctx_dicts = self.update_ndd_distribution_to_vehicle_ctx(
            ctx_dicts, modified_ndd_control_command_dicts
        )

        criticality_dicts, ctx_dicts = self.get_criticality_dicts(
            maneuver_challenge_dicts, ctx_dicts
        )

        # get the NDD distribution for each vehicle (after the collision avoidance command is added and the ndd probability is adjusted)
        ndd_control_command_dicts = {
            AgentType.VEHICLE: self.get_ndd_distribution_from_ctx(
                ctx_dicts, AgentType.VEHICLE
            ),
            AgentType.VULNERABLE_ROAD_USER: self.get_ndd_distribution_from_ctx(
                ctx_dicts, AgentType.VULNERABLE_ROAD_USER
            ),
        }
        self.step_epsilon = 1.0
        self.step_weight = 1.0
        if self.allow_NADE_IS:
            (
                ITE_control_command_dicts,
                ctx_dicts,
                weight,
                negligence_flag,
            ) = self.NADE_importance_sampling(
                ndd_control_command_dicts, maneuver_challenge_dicts, ctx_dicts
            )
            if negligence_flag:
                self.latest_IS_time = utils.get_time()
                self.allow_NADE_IS = False
        else:
            weight = 1.0
            ITE_control_command_dicts = Dict(
                {
                    AgentType.VEHICLE: {
                        veh_id: ndd_control_command_dicts[AgentType.VEHICLE][veh_id][
                            "normal"
                        ]
                        for veh_id in ndd_control_command_dicts[AgentType.VEHICLE]
                    },
                    AgentType.VULNERABLE_ROAD_USER: {
                        vru_id: ndd_control_command_dicts[
                            AgentType.VULNERABLE_ROAD_USER
                        ][vru_id]["normal"]
                        for vru_id in ndd_control_command_dicts[
                            AgentType.VULNERABLE_ROAD_USER
                        ]
                    },
                }
            )
            if utils.get_time() - self.latest_IS_time >= 2.9:
                self.allow_NADE_IS = True

        return (
            ITE_control_command_dicts,
            ctx_dicts,
            weight,
            trajectory_dicts,
            maneuver_challenge_dicts,
            criticality_dicts,
        )

    def update_ndd_distribution_to_vehicle_ctx(
        self, ctx_dicts, ndd_control_command_dicts
    ):
        for veh_id in ndd_control_command_dicts:
            ctx_dicts[AgentType.VEHICLE][veh_id][
                "ndd_command_distribution"
            ] = ndd_control_command_dicts[veh_id]
        return ctx_dicts

    def remove_collision_avoidance_command_using_avoidability(
        self, obs_dicts, trajectory_dicts, ctx_dicts
    ):
        """Remove the collision avoidance command for the vehicles that are not avoidable.

        Args:
            obs_dicts (dict): the observation dicts
            trajectory_dicts (dict): the trajectory dicts
            veh_ctx_dicts (dict): the vehicle context dicts

        Returns:
            veh_ctx_dicts (dict): the updated vehicle context dicts
        """
        potential_negligence_pair_dict = self.get_negligence_pair_dict(
            ctx_dicts, potential=True
        )
        for agent_type in [AgentType.VEHICLE, AgentType.VULNERABLE_ROAD_USER]:
            for (
                neglecting_agent_id,
                neglected_vehicle_list,
            ) in potential_negligence_pair_dict[agent_type].items():
                if (
                    ctx_dicts[agent_type][neglecting_agent_id].get("avoidable", True)
                    is False
                ):
                    for neglected_vehicle_id in neglected_vehicle_list:
                        # remove the collision avoidance command
                        ctx_dicts[AgentType.VEHICLE][neglected_vehicle_id][
                            "ndd_command_distribution"
                        ]["avoid_collision"] = None
                        trajectory_dicts[AgentType.VEHICLE][neglected_vehicle_id].pop(
                            "avoid_collision", None
                        )
                        logger.trace(
                            f"veh_id: {neglected_vehicle_id} is not avoidable from {neglecting_agent_id}, remove the collision avoidance command"
                        )
        return ctx_dicts

    def get_ndd_distribution_from_ctx(self, ctx_dicts, agent_type):
        ndd_control_command_dicts = Dict(
            {
                agent_id: ctx_dicts[agent_type][agent_id]["ndd_command_distribution"]
                for agent_id in ctx_dicts[agent_type]
            }
        )
        return ndd_control_command_dicts

    @profile
    def predict_future_trajectory_dicts(self, obs_dicts, ctx_dicts):
        # predict future trajectories for each vehicle
        sumo_net = self.simulator.sumo_net
        current_time = traci.simulation.getTime()
        trajectory_dicts = {
            AgentType.VEHICLE: {},
            AgentType.VULNERABLE_ROAD_USER: {},
        }
        # for vehicles
        ndd_control_command_dicts = self.get_ndd_distribution_from_ctx(
            ctx_dicts, AgentType.VEHICLE
        )
        for veh_id in ndd_control_command_dicts:
            obs_dict = obs_dicts[AgentType.VEHICLE][veh_id]
            veh_info = get_vehicle_info(veh_id, obs_dict, sumo_net)
            control_command_dict = ndd_control_command_dicts[veh_id]

            trajectory_dict = {
                modality: predict_future_trajectory(
                    veh_id,
                    obs_dict,
                    control_command_dict[modality],
                    sumo_net,
                    time_horizon_step=5,
                    time_resolution=0.5,
                    interpolate_resolution=0.5,
                    current_time=current_time,
                    veh_info=veh_info,
                )[0]
                for modality in control_command_dict
            }
            trajectory_dicts[AgentType.VEHICLE][veh_id] = trajectory_dict
        # for vulnerable road_users
        ndd_control_command_dicts = self.get_ndd_distribution_from_ctx(
            ctx_dicts, AgentType.VULNERABLE_ROAD_USER
        )
        for vru_id in ndd_control_command_dicts:
            obs_dict = obs_dicts[AgentType.VULNERABLE_ROAD_USER][vru_id]
            vru_info = get_vulnerbale_road_user_info(vru_id, obs_dict, sumo_net)
            control_command_dict = ndd_control_command_dicts[vru_id]
            trajectory_dict = {
                modality: predict_future_trajectory_vulnerable_road_user(
                    modality, vru_info, control_command_dict, current_time
                )
                for modality in control_command_dict
            }
            trajectory_dicts[AgentType.VULNERABLE_ROAD_USER][vru_id] = trajectory_dict
        return trajectory_dicts, ctx_dicts

    def update_control_cmds_from_predicted_trajectory(
        self, ITE_control_cmds, trajectory_dicts
    ):
        """Update the ctx_dicts with the predicted future trajectories.
        change the command type from acceleration to trajectory if the predicted collision type is rearend
        """
        for agent_type in ITE_control_cmds:
            for agent_id in ITE_control_cmds[agent_type]:
                if (
                    ITE_control_cmds[agent_type][agent_id].info.get("mode")
                    == "avoid_collision"
                    or ITE_control_cmds[agent_type][agent_id].info.get("mode")
                    == "negligence"
                    or ITE_control_cmds[agent_type][agent_id].info.get("mode")
                    == "accept_collision"
                ):
                    if (
                        ITE_control_cmds[agent_type][agent_id].command_type
                        == CommandType.ACCELERATION
                    ):
                        ITE_control_cmds[agent_type][
                            agent_id
                        ].command_type = CommandType.TRAJECTORY
                        ITE_control_cmds[agent_type][
                            agent_id
                        ].future_trajectory = trajectory_dicts[agent_type][agent_id][
                            ITE_control_cmds[agent_type][agent_id].info.get("mode")
                        ]
                        logger.info(
                            f"agent_id: {agent_id} is updated to trajectory command with mode: {ITE_control_cmds[agent_type][agent_id].info.get('mode')}, trajectory: {ITE_control_cmds[agent_type][agent_id].future_trajectory}"
                        )
        return ITE_control_cmds

    def modify_ndd_dict_according_to_avoidability(
        self, maneuver_challenge_dicts, ctx_dicts
    ):
        ndd_control_command_dicts = self.get_ndd_distribution_from_ctx(
            ctx_dicts, AgentType.VEHICLE
        )

        for veh_id in maneuver_challenge_dicts[AgentType.VEHICLE]:
            # if the vehicle negligence control command do has the potential to collide with other vehicles
            if maneuver_challenge_dicts[AgentType.VEHICLE][veh_id].get("negligence"):
                # mark all rearend collision as unavoidable
                if ctx_dicts[AgentType.VEHICLE][veh_id].get("avoidable", True) is False:
                    # collision unavoidable
                    ndd_control_command_dicts[veh_id]["negligence"].prob = (
                        ndd_control_command_dicts[veh_id]["negligence"].prob
                        * self.unavoidable_collision_prob_factor
                    )
                    ndd_control_command_dicts[veh_id]["normal"].prob = (
                        1 - ndd_control_command_dicts[veh_id]["negligence"].prob
                    )
                    logger.trace(
                        f"{veh_id} is marked as unavoidable collision and the prob is reduced to {ndd_control_command_dicts[veh_id]['negligence'].prob}"
                    )
                    self.unavoidable_maneuver_challenge_hook(veh_id)
        return ndd_control_command_dicts, ctx_dicts

    def unavoidable_maneuver_challenge_hook(self, veh_id):
        traci.vehicle.highlight(veh_id, (128, 128, 128, 255), duration=0.1)

    def get_negligence_pair_dict(self, ctx_dicts, potential=False):
        """Get the negligence pair dict.
        potential: if True, return the potential negligence pair dict, otherwise return the negligence pair dict (vehicle actually do negligence).
        """
        veh_ctx_dicts = {
            veh_id: ctx
            for veh_id, ctx in ctx_dicts[AgentType.VEHICLE].items()
            if "conflict_vehicle_list" in ctx
        }
        vru_ctx_dicts = {
            vru_id: ctx
            for vru_id, ctx in ctx_dicts[AgentType.VULNERABLE_ROAD_USER].items()
            if "conflict_vehicle_list" in ctx
        }

        if potential:
            negligence_pair_dict = {
                AgentType.VEHICLE: {
                    veh_id: ctx["conflict_vehicle_list"]
                    for veh_id, ctx in veh_ctx_dicts.items()
                },
                AgentType.VULNERABLE_ROAD_USER: {
                    vru_id: ctx["conflict_vehicle_list"]
                    for vru_id, ctx in vru_ctx_dicts.items()
                },
            }
        else:
            negligence_pair_dict = {
                AgentType.VEHICLE: {
                    veh_id: ctx["conflict_vehicle_list"]
                    for veh_id, ctx in veh_ctx_dicts.items()
                    if "mode" in ctx and ctx["mode"] == "negligence"
                },
                AgentType.VULNERABLE_ROAD_USER: {
                    vru_id: ctx["conflict_vehicle_list"]
                    for vru_id, ctx in vru_ctx_dicts.items()
                    if "mode" in ctx and ctx["mode"] == "negligence"
                },
            }
        return negligence_pair_dict

    def get_vehicle_avoidance_command(
        self,
        neglecting_vehicle_future,
        neglected_vehicle_future,
        neglecting_vehicle_id,
        neglected_vehicle_id,
    ):
        """Get the avoidance command for the neglecting vehicle.

        Args:
            neglecting_vehicle_future (list): the future trajectory of the neglecting vehicle
            neglected_vehicle_future (list): the future trajectory of the neglected vehicle

        Returns:
            avoidance_command (dict): the avoidance command
        """

        emergency_brake_deceleration = traci.vehicle.getEmergencyDecel(
            neglected_vehicle_id
        )
        avoidance_command = NDECommand(
            command_type=CommandType.ACCELERATION,
            acceleration=-emergency_brake_deceleration,
            prob=0,
            duration=3,
        )  # by default, this avoidance command prob is 0, only when really being neglected, the prob will be updated
        avoidance_command.info.update(
            {
                "mode": "avoid_collision",
                "neglecting_vehicle_id": neglecting_vehicle_id,
                "neglected_vehicle_id": neglected_vehicle_id,
            }
        )
        return avoidance_command

    def add_avoid_accept_collision_command(
        self, obs_dicts, trajectory_dicts, ctx_dicts
    ):
        potential_negligence_pair_dict = self.get_negligence_pair_dict(
            ctx_dicts, potential=True
        )
        # add avoidance command for the neglected vehicles
        for (
            neglecting_vehicle_id,
            neglected_vehicle_list,
        ) in potential_negligence_pair_dict[AgentType.VEHICLE].items():
            neglecting_vehicle_future = trajectory_dicts[AgentType.VEHICLE][
                neglecting_vehicle_id
            ]["negligence"]
            for neglected_vehicle_id in neglected_vehicle_list:
                neglected_vehicle_future = trajectory_dicts[AgentType.VEHICLE][
                    neglected_vehicle_id
                ]["normal"]
                avoidance_command = self.get_vehicle_avoidance_command(
                    neglecting_vehicle_future,
                    neglected_vehicle_future,
                    neglecting_vehicle_id,
                    neglected_vehicle_id,
                )
                ctx_dicts[AgentType.VEHICLE][neglected_vehicle_id][
                    "ndd_command_distribution"
                ]["avoid_collision"] = avoidance_command
                (
                    trajectory_dicts[AgentType.VEHICLE][neglected_vehicle_id][
                        "avoid_collision"
                    ],
                    info,
                ) = predict_future_trajectory(
                    neglected_vehicle_id,
                    obs_dicts[AgentType.VEHICLE][neglected_vehicle_id],
                    avoidance_command,
                    self.simulator.sumo_net,
                    time_horizon_step=5,
                    time_resolution=0.5,
                    interpolate_resolution=0.5,
                    current_time=None,
                    veh_info=None,
                )

                accept_command = self.get_accept_collision_command()
                ctx_dicts[AgentType.VEHICLE][neglected_vehicle_id][
                    "ndd_command_distribution"
                ]["accept_collision"] = accept_command
                (
                    trajectory_dicts[AgentType.VEHICLE][neglected_vehicle_id][
                        "accept_collision"
                    ],
                    info,
                ) = predict_future_trajectory(
                    neglected_vehicle_id,
                    obs_dicts[AgentType.VEHICLE][neglected_vehicle_id],
                    accept_command,
                    self.simulator.sumo_net,
                    time_horizon_step=5,
                    time_resolution=0.5,
                    interpolate_resolution=0.5,
                    current_time=None,
                    veh_info=None,
                )

                logger.trace(
                    f"add avoidance command for vehicle: {neglected_vehicle_id}, with info {info}"
                )
            logger.trace(
                f"add avoidance command for vehicle: {neglected_vehicle_list} from vehicle: {neglecting_vehicle_id}"
            )
        # add avoidance command for the neglected vulnerable road users
        # TODO: if the neglected vehicle already has the avoidance command, do not add the avoidance command again
        for (
            neglecting_vru_id,
            neglected_vehicle_list,
        ) in potential_negligence_pair_dict[AgentType.VULNERABLE_ROAD_USER].items():
            neglecting_vru_future = trajectory_dicts[AgentType.VULNERABLE_ROAD_USER][
                neglecting_vru_id
            ]["negligence"]
            for neglected_vehicle_id in neglected_vehicle_list:
                if (
                    ctx_dicts[AgentType.VEHICLE][neglected_vehicle_id][
                        "ndd_command_distribution"
                    ].get("avoid_collision", None)
                    is not None
                ):
                    continue
                neglected_vehicle_future = trajectory_dicts[AgentType.VEHICLE][
                    neglected_vehicle_id
                ]["normal"]
                avoidance_command = self.get_vehicle_avoidance_command(
                    neglecting_vru_future,
                    neglected_vehicle_future,
                    neglecting_vru_id,
                    neglected_vehicle_id,
                )
                ctx_dicts[AgentType.VEHICLE][neglected_vehicle_id][
                    "ndd_command_distribution"
                ]["avoid_collision"] = avoidance_command
                (
                    trajectory_dicts[AgentType.VEHICLE][neglected_vehicle_id][
                        "avoid_collision"
                    ],
                    info,
                ) = predict_future_trajectory(
                    neglected_vehicle_id,
                    obs_dicts[AgentType.VEHICLE][neglected_vehicle_id],
                    avoidance_command,
                    self.simulator.sumo_net,
                    time_horizon_step=5,
                    time_resolution=0.5,
                    interpolate_resolution=0.5,
                    current_time=None,
                    veh_info=None,
                )

                accept_command = self.get_accept_collision_command()
                ctx_dicts[AgentType.VEHICLE][neglected_vehicle_id][
                    "ndd_command_distribution"
                ]["accept_collision"] = accept_command
                (
                    trajectory_dicts[AgentType.VEHICLE][neglected_vehicle_id][
                        "accept_collision"
                    ],
                    info,
                ) = predict_future_trajectory(
                    neglected_vehicle_id,
                    obs_dicts[AgentType.VEHICLE][neglected_vehicle_id],
                    accept_command,
                    self.simulator.sumo_net,
                    time_horizon_step=5,
                    time_resolution=0.5,
                    interpolate_resolution=0.5,
                    current_time=None,
                    veh_info=None,
                )

                logger.trace(
                    f"add avoidance command for vehicle: {neglected_vehicle_id}, with info {info}"
                )
            logger.trace(
                f"add avoidance command for vehicle: {neglected_vehicle_list} from vru: {neglecting_vru_id}"
            )
        return trajectory_dicts, ctx_dicts

    def record_negligence_related_information(self, negligence_pair_dict, ctx_dicts):
        if len(negligence_pair_dict[AgentType.VEHICLE]):
            self.record.event_info[
                utils.get_time()
            ].negligence_pair_dict = negligence_pair_dict[AgentType.VEHICLE]
            self.record.event_info[utils.get_time()].neglecting_vehicle_id = list(
                negligence_pair_dict[AgentType.VEHICLE].keys()
            )[0]
            negligence_command_dict = {
                veh_id: ctx_dicts[AgentType.VEHICLE][
                    veh_id
                ].ndd_command_distribution.negligence
                for veh_id in negligence_pair_dict[AgentType.VEHICLE]
            }

            neglected_vehicle_id_set = set()
            for neglected_vehicle_list in negligence_pair_dict[
                AgentType.VEHICLE
            ].values():
                neglected_vehicle_id_set.update(neglected_vehicle_list)

            neglected_command_dict = {
                veh_id: ctx_dicts[AgentType.VEHICLE][veh_id].ndd_command_distribution
                for veh_id in neglected_vehicle_id_set
            }

            self.record.event_info[utils.get_time()].negligence_info_dict = {
                veh_id: negligence_command.info
                for veh_id, negligence_command in negligence_command_dict.items()
            }

            negligence_command_dict = {
                veh_id: str(negligence_command)
                for veh_id, negligence_command in negligence_command_dict.items()
            }

            neglected_command_dict = {
                veh_id: str(neglected_command)
                for veh_id, neglected_command in neglected_command_dict.items()
            }

            self.record.event_info[
                utils.get_time()
            ].negligence_command = negligence_command_dict
            self.record.event_info[
                utils.get_time()
            ].neglected_command = neglected_command_dict
        return ctx_dicts

    def apply_collision_avoidance(
        self,
        trajectory_dicts,
        ctx_dicts,
        ITE_control_command_dict,
    ):
        """after the NADE decision, apply collision avoidance for the neglected vehicles.

        Args:
            ctx_dicts (_type_): _description_
            ITE_control_command_dict (_type_): _description_

        Returns:
            _type_: _description_
        """

        negligence_pair_dict = self.get_negligence_pair_dict(ctx_dicts)
        avoid_collision_IS_prob = float(os.getenv("AVOID_COLLISION_IS_PROB", 0.2))
        avoid_collision_ndd_prob = 0.99
        weight = 1.0
        ctx_dicts = self.record_negligence_related_information(
            negligence_pair_dict, ctx_dicts
        )
        # no vehicle neglected
        if len(negligence_pair_dict[AgentType.VEHICLE]) == 0:
            return ITE_control_command_dict, ctx_dicts, weight

        # neglected vehicle set is all the vehicles that are neglected by the neglecting vehicle, combine all vehicles in the negligence_pair_dict values
        neglected_vehicle_set = set()
        for neglected_vehicle_list in negligence_pair_dict[AgentType.VEHICLE].values():
            neglected_vehicle_set.update(neglected_vehicle_list)

        avoidance_command_list = [
            ctx_dicts[AgentType.VEHICLE][veh_id]["ndd_command_distribution"].get(
                "avoid_collision", None
            )
            for veh_id in neglected_vehicle_set
        ]
        # no collision avoidance can be applied (predicted not avoidable)
        if all(
            avoidance_command is None for avoidance_command in avoidance_command_list
        ):
            logger.critical(
                f"all avoidance command is None, no collision avoidance command will be selected and NADE for collision avoidance will be disabled, neglected_vehicle_set: {neglected_vehicle_set}"
            )
            for neglected_vehicle_id in neglected_vehicle_set:
                ctx_dicts[AgentType.VEHICLE][neglected_vehicle_id][
                    "mode"
                ] = "accept_collision"
                self.record.event_info[utils.get_time()].update(
                    {
                        "neglected_vehicle_id": neglected_vehicle_id,
                        "mode": "accept_collision",
                        "additional_info": "all_avoidance_none",
                    }
                )
                ITE_control_command_dict[AgentType.VEHICLE][
                    neglected_vehicle_id
                ] = ctx_dicts[AgentType.VEHICLE][neglected_vehicle_id][
                    "ndd_command_distribution"
                ].get(
                    "accept_collision", None
                )
            return ITE_control_command_dict, ctx_dicts, weight

        timestamp = utils.get_time()
        IS_prob = np.random.uniform(0, 1)

        # avoid collision
        if IS_prob < avoid_collision_IS_prob:
            for (
                neglecting_vehicle_id,
                neglected_vehicle_list,
            ) in negligence_pair_dict[AgentType.VEHICLE].items():
                if neglecting_vehicle_id == "CAV":
                    logger.critical(
                        f"neglecting_vehicle_id: {neglecting_vehicle_id}, neglected_vehicle_list: {neglected_vehicle_list}"
                    )
                logger.info(
                    f"{timestamp}, neglected_vehicle_list: {neglected_vehicle_list} avoiding collision from {neglecting_vehicle_id}, avoidability: {ctx_dicts[AgentType.VEHICLE][neglecting_vehicle_id].get('avoidable', True)}"
                )
                for neglected_vehicle_id in neglected_vehicle_list:
                    avoid_collision_command = ctx_dicts[AgentType.VEHICLE][
                        neglected_vehicle_id
                    ]["ndd_command_distribution"].get("avoid_collision", None)
                    # if avoidable, then collision command should be available, if not avoidable, then collision command should be None
                    assert (
                        (avoid_collision_command is not None)
                        and (
                            ctx_dicts[AgentType.VEHICLE][neglecting_vehicle_id][
                                "avoidable"
                            ]
                        )
                    ) or (
                        (avoid_collision_command is None)
                        and (
                            not ctx_dicts[AgentType.VEHICLE][neglecting_vehicle_id][
                                "avoidable"
                            ]
                        )
                    )
                    if avoid_collision_command:
                        ITE_control_command_dict[AgentType.VEHICLE][
                            neglected_vehicle_id
                        ] = avoid_collision_command
                        ctx_dicts[AgentType.VEHICLE][neglected_vehicle_id][
                            "mode"
                        ] = "avoid_collision"
                        self.record.event_info[utils.get_time()].update(
                            {
                                "neglected_vehicle_id": neglected_vehicle_id,
                                "mode": "avoid_collision",
                            }
                        )
                    else:
                        logger.critical(
                            f"neglected_vehicle_id: {neglected_vehicle_id} does not have avoidance command from {neglecting_vehicle_id}, avoidability: {ctx_dicts[AgentType.VEHICLE][neglecting_vehicle_id].get('avoidable', True)}"
                        )
            weight *= avoid_collision_ndd_prob / avoid_collision_IS_prob
        # accept collision
        else:
            for (
                neglecting_vehicle_id,
                neglected_vehicle_list,
            ) in negligence_pair_dict[AgentType.VEHICLE].items():
                logger.info(
                    f"{timestamp}, neglected_vehicle_list: {neglected_vehicle_list} accept collision from {neglecting_vehicle_id}, avoidability: {ctx_dicts[AgentType.VEHICLE][neglecting_vehicle_id].get('avoidable', True)}"
                )
                for neglected_vehicle_id in neglected_vehicle_list:
                    ctx_dicts[AgentType.VEHICLE][neglected_vehicle_id][
                        "mode"
                    ] = "accept_collision"
                    self.record.event_info[utils.get_time()].update(
                        {
                            "neglected_vehicle_id": neglected_vehicle_id,
                            "mode": "accept_collision",
                        }
                    )
                    ITE_control_command_dict[AgentType.VEHICLE][
                        neglected_vehicle_id
                    ] = ctx_dicts[AgentType.VEHICLE][neglected_vehicle_id][
                        "ndd_command_distribution"
                    ].get(
                        "accept_collision", None
                    )
            weight *= (1 - avoid_collision_ndd_prob) / (1 - avoid_collision_IS_prob)

        self.record.event_info[utils.get_time()].neglected_command = {
            str(ITE_control_command_dict[neglected_vehicle_id])
            for neglected_vehicle_id in neglected_vehicle_set
        }

        return ITE_control_command_dict, ctx_dicts, weight

    def get_accept_collision_command(self):
        accept_command = NDECommand(
            command_type=CommandType.ACCELERATION,
            acceleration=0,
            prob=0,
            duration=2,
        )  # the vehicle will accept the final collision
        accept_command.info = {"mode": "accept_collision"}
        return accept_command

    def get_neglecting_vehicle_id(self, control_command_dict, maneuver_challenge_info):
        neglect_pair_list = []
        for veh_id in control_command_dict:
            control_command = control_command_dict[veh_id]
            if "mode" in control_command and control_command["mode"] == "negligence":
                neglecting_vehicle_id = veh_id
                neglected_vehicle_id = list(
                    maneuver_challenge_info[neglecting_vehicle_id].keys()
                )
                neglect_pair_list.append((neglecting_vehicle_id, neglected_vehicle_id))
        return neglect_pair_list

    def NADE_importance_sampling(
        self,
        ndd_control_command_dicts,
        maneuver_challenge_dicts,
        ctx_dicts,
        exclude_IS_agent_set=None,
    ):
        """Importance sampling for NADE.

        Args:
            ndd_control_command_dict (dict): for each vehicle v_id, ndd_control_command_dict[veh_id] = ndd_control_command, ndd_pdf
            criticality_dict (dict): for each vehicle v_id, criticality_dict[veh_id] = criticality

        Returns:
            weight (float): the importance sampling weight
        """
        weight = 1.0
        epsilon = 1.0
        # intialize the ITE control command dict with the same keys as the ndd_control_command_dict
        ITE_control_command_dict = Dict(
            {
                AgentType.VEHICLE: {
                    veh_id: ndd_control_command_dicts[AgentType.VEHICLE][veh_id][
                        "normal"
                    ]
                    for veh_id in ndd_control_command_dicts[AgentType.VEHICLE]
                },
                AgentType.VULNERABLE_ROAD_USER: {
                    vru_id: ndd_control_command_dicts[AgentType.VULNERABLE_ROAD_USER][
                        vru_id
                    ]["normal"]
                    for vru_id in ndd_control_command_dicts[
                        AgentType.VULNERABLE_ROAD_USER
                    ]
                },
            }
        )
        negligence_flag = False
        exclude_IS_agent_set = (
            set() if exclude_IS_agent_set is None else exclude_IS_agent_set
        )

        for agent_type in [AgentType.VEHICLE, AgentType.VULNERABLE_ROAD_USER]:
            for agent_id in maneuver_challenge_dicts[agent_type]:
                if agent_id in exclude_IS_agent_set:
                    continue
                if maneuver_challenge_dicts[agent_type][agent_id].get("negligence"):
                    ndd_normal_prob = ndd_control_command_dicts[agent_type][
                        agent_id
                    ].normal.prob
                    ndd_negligence_prob = ndd_control_command_dicts[agent_type][
                        agent_id
                    ].negligence.prob
                    assert (
                        ndd_normal_prob + ndd_negligence_prob == 1
                    ), "The sum of the probabilities of the normal and negligence control commands should be 1."

                    # get the importance sampling probability
                    IS_prob = self.get_IS_prob(
                        agent_id,
                        ndd_control_command_dicts[agent_type],
                        maneuver_challenge_dicts[agent_type],
                        ctx_dicts[agent_type],
                    )
                    epsilon = 1 - IS_prob

                    # update the importance sampling weight and the ITE control command
                    sampled_prob = np.random.uniform(0, 1)
                    if sampled_prob < IS_prob:  # select the negligece control command
                        weight *= ndd_negligence_prob / IS_prob
                        ITE_control_command_dict[agent_type][
                            agent_id
                        ] = ndd_control_command_dicts[agent_type][agent_id].negligence
                        ctx_dicts[agent_type][agent_id]["mode"] = "negligence"
                        if agent_type == AgentType.VEHICLE:
                            self.negligence_hook(agent_id)
                        logger.info(
                            f"time: {utils.get_time()}, agent_id: {agent_id} select negligence control command, IS_prob: {IS_prob}, ndd_prob: {ndd_negligence_prob}, weight: {self.importance_sampling_weight}"
                        )
                        negligence_flag = True
                    else:
                        weight *= ndd_normal_prob / (1 - IS_prob)
                        ITE_control_command_dict[agent_type][
                            agent_id
                        ] = ndd_control_command_dicts[agent_type][agent_id]["normal"]
                        logger.trace(
                            f"time: {utils.get_time()}, agent_id: {agent_id} select normal control command, IS_prob: {IS_prob}, weight: {self.importance_sampling_weight}"
                        )
        self.step_epsilon = epsilon
        self.step_weight = weight
        return ITE_control_command_dict, ctx_dicts, weight, negligence_flag

    def negligence_hook(self, veh_id):
        traci.vehicle.highlight(veh_id, (255, 0, 0, 255), duration=2)

    def get_IS_prob(
        self, agent_id, ndd_control_command_dicts, maneuver_challenge_dicts, ctx_dicts
    ):
        if not maneuver_challenge_dicts[agent_id].get("negligence"):
            raise ValueError("The vehicle is not in the negligence mode.")

        IS_magnitude = IS_MAGNITUDE_DEFAULT
        try:
            predicted_collision_type = ndd_control_command_dicts[
                agent_id
            ].negligence.info["predicted_collision_type"]

            # get the importance sampling magnitude according to the predicted collision type
            for collision_type, env_var in IS_MAGNITUDE_MAPPING.items():
                if collision_type in predicted_collision_type:
                    IS_magnitude = float(os.getenv(env_var, IS_magnitude))
                    break

            # if the vehicle is not avoidable, increase the importance sampling magnitude
            if not ctx_dicts[agent_id].get("avoidable", True):
                IS_magnitude *= IS_MAGNITUDE_MULTIPLIER
            # logger.trace(f"IS_magnitude: {IS_magnitude} for {collision_type}")
            # logger.trace(f"Original prob: {ndd_control_command_dicts[veh_id]["negligence"].prob}")
            # final_is_prob = np.clip(
            #     ndd_control_command_dicts[veh_id]["negligence"].prob * IS_magnitude,
            #     0,
            #     self.max_importance_sampling_prob,
            # )
            # logger.trace(f"final IS prob for veh_id: {final_is_prob}")

        except Exception as e:
            logger.critical(f"Error in getting the importance sampling magnitude: {e}")

        return np.clip(
            ndd_control_command_dicts[agent_id]["negligence"].prob * IS_magnitude,
            0,
            self.max_importance_sampling_prob,
        )

    def get_avoidability_dicts(
        self, maneuver_challenge_dicts, trajectory_dicts, obs_dicts, ctx_dicts
    ):
        negligence_future_trajectory_dict = Dict(
            {
                AgentType.VEHICLE: {
                    veh_id: trajectory_dicts[AgentType.VEHICLE][veh_id].get(
                        "negligence", None
                    )
                    for veh_id in trajectory_dicts[AgentType.VEHICLE]
                },
                AgentType.VULNERABLE_ROAD_USER: {
                    vru_id: trajectory_dicts[AgentType.VULNERABLE_ROAD_USER][
                        vru_id
                    ].get("negligence", None)
                    for vru_id in trajectory_dicts[AgentType.VULNERABLE_ROAD_USER]
                },
            }
        )
        avoidance_future_trajectory_dict = Dict(
            {
                veh_id: trajectory_dicts[AgentType.VEHICLE][veh_id].get(
                    "avoid_collision", None
                )
                for veh_id in trajectory_dicts[AgentType.VEHICLE]
            }
        )

        # initialize the avoidability of each vehicle
        for veh_id in maneuver_challenge_dicts[AgentType.VEHICLE]:
            ctx_dicts[AgentType.VEHICLE][veh_id]["avoidable"] = True

        # get the maneuver challenge for the negligence vehicle future and the avoidance vehicle future
        maneuver_challenge_avoidance_dicts = Dict(
            {
                AgentType.VEHICLE: {},
                AgentType.VULNERABLE_ROAD_USER: {},
            }
        )
        for veh_id in maneuver_challenge_dicts[AgentType.VEHICLE]:
            if maneuver_challenge_dicts[AgentType.VEHICLE][veh_id].get("negligence"):
                conflict_vehicle_list = ctx_dicts[AgentType.VEHICLE][veh_id].get(
                    "conflict_vehicle_list", []
                )
                conflict_vehicle_future_dict = Dict(
                    {
                        veh_id: avoidance_future_trajectory_dict[veh_id]
                        for veh_id in conflict_vehicle_list
                    }
                )
                maneuver_challenge_avoidance_dicts[AgentType.VEHICLE][
                    veh_id
                ] = self.get_maneuver_challenge(
                    veh_id,
                    negligence_future_trajectory_dict[AgentType.VEHICLE][veh_id],
                    AgentType.VEHICLE,
                    conflict_vehicle_future_dict,
                    AgentType.VEHICLE,
                    obs_dicts,
                    ctx_dicts[AgentType.VEHICLE][veh_id],
                    record_in_ctx=False,
                    buffer=0.5,  # buffer for the collision avoidance, 1m
                )
                if maneuver_challenge_avoidance_dicts[AgentType.VEHICLE][veh_id].get(
                    "negligence"
                ):
                    ctx_dicts[AgentType.VEHICLE][veh_id]["avoidable"] = False
                    logger.debug(
                        f"timestep: {utils.get_time()}, veh_id: {veh_id} is not avoidable"
                    )
                else:
                    logger.debug(
                        f"timestep: {utils.get_time()}, veh_id: {veh_id} is avoidable"
                    )
                logger.trace(
                    f"negligence vehicle observation {obs_dicts[AgentType.VEHICLE][veh_id]}, conflict vehicle observation {Dict({veh_id: obs_dicts[AgentType.VEHICLE][veh_id] for veh_id in conflict_vehicle_list})}"
                )
                logger.trace(
                    f"negligence future trajectory dict for {veh_id}: {negligence_future_trajectory_dict[AgentType.VEHICLE][veh_id]}, and conflict future trajectory dict for {conflict_vehicle_list}: {conflict_vehicle_future_dict}"
                )
        for vru_id in maneuver_challenge_dicts[AgentType.VULNERABLE_ROAD_USER]:
            if maneuver_challenge_dicts[AgentType.VULNERABLE_ROAD_USER][vru_id].get(
                "negligence"
            ):
                conflict_vehicle_list = ctx_dicts[AgentType.VULNERABLE_ROAD_USER][
                    vru_id
                ].get("conflict_vehicle_list", [])
                conflict_vehicle_future_dict = Dict(
                    {
                        veh_id: avoidance_future_trajectory_dict[veh_id]
                        for veh_id in conflict_vehicle_list
                    }
                )
                maneuver_challenge_avoidance_dicts[AgentType.VULNERABLE_ROAD_USER][
                    vru_id
                ] = self.get_maneuver_challenge(
                    vru_id,
                    negligence_future_trajectory_dict[AgentType.VULNERABLE_ROAD_USER][
                        vru_id
                    ],
                    AgentType.VULNERABLE_ROAD_USER,
                    conflict_vehicle_future_dict,
                    AgentType.VEHICLE,
                    obs_dicts,
                    ctx_dicts[AgentType.VULNERABLE_ROAD_USER][vru_id],
                    record_in_ctx=False,
                    buffer=0.5,  # buffer for the collision avoidance, 1m
                )
                if maneuver_challenge_avoidance_dicts[AgentType.VULNERABLE_ROAD_USER][
                    vru_id
                ].get("negligence"):
                    ctx_dicts[AgentType.VULNERABLE_ROAD_USER][vru_id][
                        "avoidable"
                    ] = False
                    logger.debug(
                        f"timestep: {utils.get_time()}, vru_id: {vru_id} is not avoidable"
                    )
                else:
                    logger.debug(
                        f"timestep: {utils.get_time()}, vru_id: {vru_id} is avoidable"
                    )
                logger.trace(
                    f"negligence vru observation {obs_dicts[AgentType.VULNERABLE_ROAD_USER][vru_id]}, conflict vehicle observation {Dict({veh_id: obs_dicts[AgentType.VEHICLE][veh_id] for veh_id in conflict_vehicle_list})}"
                )
                logger.trace(
                    f"negligence future trajectory dict for {vru_id}: {negligence_future_trajectory_dict[AgentType.VULNERABLE_ROAD_USER][vru_id]}, and conflict future trajectory dict for {conflict_vehicle_list}: {conflict_vehicle_future_dict}"
                )

        return maneuver_challenge_avoidance_dicts, ctx_dicts

    @profile
    def get_maneuver_challenge_dicts(self, trajectory_dicts, obs_dicts, ctx_dicts):
        """Get the maneuver challenge for each vehicle when it is in the negligence mode while other vehicles are in the normal mode.
        Note: We only consider the challenge for the following cases:
        1. vehicle in the negligence mode and the vehicle in the normal mode
        2. vru in the negligence mode and the vehicle in the normal mode

        Args:
            trajectory_dicts (dict): trajectory_dicts[veh_id] = {"normal": normal_future_trajectory, "negligence": negligence_future_trajectory}

        Returns:
            maneuver_challenge_dict (dict): maneuver_challenge_dict[veh_id] = (num_affected_vehicles, affected_vehicles)
        """
        normal_future_trajectory_dict_veh = Dict(
            {
                veh_id: trajectory_dicts[AgentType.VEHICLE][veh_id].get("normal", None)
                for veh_id in trajectory_dicts[AgentType.VEHICLE]
            }
        )
        negligence_future_trajectory_dict_veh = Dict(
            {
                veh_id: trajectory_dicts[AgentType.VEHICLE][veh_id].get(
                    "negligence", None
                )
                for veh_id in trajectory_dicts[AgentType.VEHICLE]
            }
        )
        negligence_future_trajectory_dict_vru = Dict(
            {
                vru_id: trajectory_dicts[AgentType.VULNERABLE_ROAD_USER][vru_id].get(
                    "negligence", None
                )
                for vru_id in trajectory_dicts[AgentType.VULNERABLE_ROAD_USER]
            }
        )

        # get the maneuver challenge for each vehicle, check if the negligence future will collide with other vehicles' normal future
        maneuver_challenge_dicts_veh = Dict(
            {
                veh_id: self.get_maneuver_challenge(
                    veh_id,
                    negligence_future_trajectory_dict_veh[veh_id],
                    AgentType.VEHICLE,
                    normal_future_trajectory_dict_veh,
                    AgentType.VEHICLE,
                    obs_dicts,
                    ctx_dicts[AgentType.VEHICLE][veh_id],
                    record_in_ctx=True,
                )
                for veh_id in trajectory_dicts[AgentType.VEHICLE]
            }
        )
        maneuver_challenge_dicts_vru = Dict(
            {
                vru_id: self.get_maneuver_challenge(
                    vru_id,
                    negligence_future_trajectory_dict_vru[vru_id],
                    AgentType.VULNERABLE_ROAD_USER,
                    normal_future_trajectory_dict_veh,
                    AgentType.VEHICLE,
                    obs_dicts,
                    ctx_dicts[AgentType.VULNERABLE_ROAD_USER][vru_id],
                    record_in_ctx=True,
                )
                for vru_id in trajectory_dicts[AgentType.VULNERABLE_ROAD_USER]
            }
        )

        for veh_id in ctx_dicts[AgentType.VEHICLE]:
            ctx_dicts[AgentType.VEHICLE][veh_id]["maneuver_challenge"] = (
                maneuver_challenge_dicts_veh[veh_id]
                if veh_id in maneuver_challenge_dicts_veh
                else {"normal": 0}
            )

        for vru_id in ctx_dicts[AgentType.VULNERABLE_ROAD_USER]:
            ctx_dicts[AgentType.VULNERABLE_ROAD_USER][vru_id]["maneuver_challenge"] = (
                maneuver_challenge_dicts_vru[vru_id]
                if vru_id in maneuver_challenge_dicts_vru
                else {"normal": 0}
            )

        maneuver_challenge_dicts_veh_shrinked = Dict(
            {
                veh_id: maneuver_challenge_dicts_veh[veh_id]
                for veh_id in maneuver_challenge_dicts_veh
                if maneuver_challenge_dicts_veh[veh_id].get("negligence")
            }
        )
        for veh_id in maneuver_challenge_dicts_veh_shrinked:
            self.avoidable_maneuver_challenge_hook(veh_id)
        conflict_vehicle_info = Dict(
            {
                AgentType.VEHICLE: {
                    veh_id: ctx_dicts[AgentType.VEHICLE][veh_id].get(
                        "conflict_vehicle_list"
                    )
                    for veh_id in ctx_dicts[AgentType.VEHICLE]
                    if ctx_dicts[AgentType.VEHICLE][veh_id].get("conflict_vehicle_list")
                },
                AgentType.VULNERABLE_ROAD_USER: {
                    vru_id: ctx_dicts[AgentType.VULNERABLE_ROAD_USER][vru_id].get(
                        "conflict_vehicle_list"
                    )
                    for vru_id in ctx_dicts[AgentType.VULNERABLE_ROAD_USER]
                    if ctx_dicts[AgentType.VULNERABLE_ROAD_USER][vru_id].get(
                        "conflict_vehicle_list"
                    )
                },
            }
        )
        logger.trace(
            f"maneuver_challenge: {maneuver_challenge_dicts_veh_shrinked}, conflict_vehicle_info: {conflict_vehicle_info}"
        )
        maneuver_challenge_dicts = {
            AgentType.VEHICLE: maneuver_challenge_dicts_veh,
            AgentType.VULNERABLE_ROAD_USER: maneuver_challenge_dicts_vru,
        }
        return maneuver_challenge_dicts, ctx_dicts

    def avoidable_maneuver_challenge_hook(self, veh_id):
        traci.vehicle.highlight(veh_id, (255, 0, 0, 120), duration=0.1)

    def get_maneuver_challenge(
        self,
        negligence_agent_id,
        negligence_agent_future,
        negligence_agent_type,
        all_normal_agent_future,
        normal_agent_type,
        obs_dicts,
        agent_ctx_dict,
        record_in_ctx=False,
        highlight_flag=True,
        buffer=0,
    ):
        """Get the maneuver challenge for the negligence vehicle.

        Args:
            negligence_agent_id (str): the id of the negligence agent
            negligence_agent_future (list): the future trajectory of the negligence agent
            negligence_agent_type (str): "vehicle" or "vulnerable_road_user"
            all_normal_agent_future (dict): all_normal_veh _future[veh_id] = future trajectory of the normal agent
            normal_agent_type (str): "vehicle" or "vulnerable_road_user"

        Returns:
            num_affected_vehicles (int): the number of vehicles that will be affected by the negligence vehicle
            maneuver_challenge_info (dict): maneuver_challenge_info[veh_id] = 1 if the negligence vehicle will affect the normal vehicle
        """
        # see if the one negligence future will intersect with other normal futures
        final_collision_flag = False
        if negligence_agent_future is not None and all_normal_agent_future is not None:
            for agent_id in all_normal_agent_future:
                if agent_id == negligence_agent_id:
                    continue
                if all_normal_agent_future[agent_id] is None:
                    print(
                        f"agent_id: {agent_id}, all_normal_agent_future[agent_id]: {all_normal_agent_future[agent_id]}"
                    )
                link_intersection_flag = is_link_intersect(
                    obs_dicts[negligence_agent_type][negligence_agent_id],
                    obs_dicts[normal_agent_type][agent_id],
                )
                if not link_intersection_flag:
                    continue  # if the next link of the two vehicles are not intersected, then the two vehicles will not collide

                # if the negligence_veh_id is the header of the normal_veh_id, then the two vehicles will not collide
                # TODO: traci.vehicle.getLeader() can not detect the leading vulnerable road user
                if (
                    normal_agent_type == AgentType.VEHICLE
                    and negligence_agent_type == AgentType.VEHICLE
                ):
                    leader = traci.vehicle.getLeader(agent_id)
                    if leader is not None and leader[0] == negligence_agent_id:
                        continue

                # collision_flag = is_intersect(
                #     negligence_agent_future,
                #     all_normal_agent_future[agent_id],
                #     veh_length,
                #     tem_len,
                #     circle_r + buffer,
                # ) # TODO: consider different vehicle length between the two colliding vehicles or vehicle and vulnerable road user
                collision_flag = is_intersect(
                    negligence_agent_future,
                    all_normal_agent_future[agent_id],
                    obs_dicts[negligence_agent_type][negligence_agent_id]["ego"][
                        "length"
                    ],
                    obs_dicts[normal_agent_type][agent_id]["ego"]["length"],
                    obs_dicts[negligence_agent_type][negligence_agent_id]["ego"][
                        "width"
                    ],
                    obs_dicts[normal_agent_type][agent_id]["ego"]["width"],
                    negligence_agent_type.value,
                    normal_agent_type.value,
                    buffer,
                )
                final_collision_flag = final_collision_flag or collision_flag
                if collision_flag and record_in_ctx:
                    if "conflict_vehicle_list" not in agent_ctx_dict:
                        agent_ctx_dict["conflict_vehicle_list"] = []
                    agent_ctx_dict["conflict_vehicle_list"].append(agent_id)
                    # logger.trace(
                    #     f"veh_id: {negligence_veh_id} will collide with veh_id: {veh_id}"
                    # )
            return {
                "normal": 0,
                "negligence": (1 if final_collision_flag else 0),
            }  # the first element is the number of vehicles that will be affected by the negligence vehicle
        else:
            return {"normal": 0}

    def get_criticality_dicts(self, maneuver_challenge_dicts, ctx_dicts):
        ndd_control_command_dicts = self.get_ndd_distribution_from_ctx(
            ctx_dicts, AgentType.VEHICLE
        )
        criticality_dicts = {}
        for veh_id in maneuver_challenge_dicts[AgentType.VEHICLE]:
            ndd_control_command_dict = ndd_control_command_dicts[veh_id]
            maneuver_challenge_dict = maneuver_challenge_dicts[AgentType.VEHICLE][
                veh_id
            ]
            criticality_dicts[veh_id] = {
                modality: ndd_control_command_dict[modality].prob
                * maneuver_challenge_dict[modality]
                for modality in maneuver_challenge_dict
                if modality != "info"
            }
        for veh_id in ctx_dicts[AgentType.VEHICLE]:
            ctx_dicts[AgentType.VEHICLE][veh_id]["criticality"] = (
                criticality_dicts[veh_id]
                if veh_id in criticality_dicts
                else {"normal": 0}
            )
        return criticality_dicts, ctx_dicts


def is_link_intersect(veh1_obs, veh2_obs):
    veh_1_edge_id = veh1_obs["ego"]["edge_id"]
    veh_2_edge_id = veh2_obs["ego"]["edge_id"]
    if veh_1_edge_id == veh_2_edge_id:
        return True

    veh1_next_lane_id_set = set(veh1_obs["ego"]["upcoming_lanes"])
    veh2_next_lane_id_set = set(veh2_obs["ego"]["upcoming_lanes"])

    if veh1_next_lane_id_set.intersection(veh2_next_lane_id_set):
        return True

    veh1_foe_lane_id_set = set(veh1_obs["ego"]["upcoming_foe_lane_id_list"])
    veh2_foe_lane_id_set = set(veh2_obs["ego"]["upcoming_foe_lane_id_list"])

    # if the next lane of the two vehicles are intersected
    if veh1_foe_lane_id_set.intersection(
        veh2_next_lane_id_set
    ) or veh2_foe_lane_id_set.intersection(veh1_next_lane_id_set):
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
    if control_command["longitudinal"] == "SUMO":
        return 0
    return control_command["longitudinal"]
