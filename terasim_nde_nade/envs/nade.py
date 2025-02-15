import json
import math
import os
import numpy as np
from addict import Dict
from loguru import logger

from terasim.envs.template_complete import EnvTemplateComplete
from terasim.overlay import profile, traci
from terasim.params import AgentType
import terasim.utils as utils
from terasim_nde_nade.envs.nde import NDE
from terasim_nde_nade.utils import (
    CommandType,
)
from terasim_nde_nade.envs.utils.trajectory_prediction import (
    predict_future_trajectory_dicts,
    get_criticality_dicts,
    get_ndd_distribution_from_ctx,
    update_ndd_distribution_to_vehicle_ctx,
    update_control_cmds_from_predicted_trajectory,
)
from terasim_nde_nade.envs.utils.maneuver_challenge import get_maneuver_challenge_dicts
from terasim_nde_nade.envs.utils.avoidance import (
    add_avoid_accept_collision_command,
    get_avoidability_dicts,
    modify_ndd_dict_according_to_avoidability,
    remove_collision_avoidance_command_using_avoidability,
    apply_collision_avoidance,
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
        ITE_control_cmds, ctx_dicts, weight = apply_collision_avoidance(
            trajectory_dicts, ctx_dicts, ITE_control_cmds, self.record
        )
        self.importance_sampling_weight *= (
            weight  # update weight by collision avoidance
        )
        ITE_control_cmds = update_control_cmds_from_predicted_trajectory(
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

    @profile
    def NADE_decision(self, control_command_dicts, ctx_dicts, obs_dicts):
        """NADE decision here.

        Args:
            control_command_dicts
            obs_dicts
        """
        trajectory_dicts, ctx_dicts = predict_future_trajectory_dicts(
            self.simulator.sumo_net, obs_dicts, ctx_dicts, 
        )  # predict future trajectories for each vehicle (normal/negligence)

        # get maneuver challenge and collision avoidability
        # will mark the conflict vehicles in the veh_ctx_dicts
        maneuver_challenge_dicts, ctx_dicts = get_maneuver_challenge_dicts(
            trajectory_dicts,
            obs_dicts,
            ctx_dicts,
        )

        # add collision avoidance command for the neglected vehicles, and predict the future trajectories for the avoidance command using the veh_ctx_dicts
        trajectory_dicts, ctx_dicts = add_avoid_accept_collision_command(
            self.simulator.sumo_net, obs_dicts, trajectory_dicts, ctx_dicts
        )

        # update the ndd probability according to collision avoidability
        avoidability_dicts, ctx_dicts = get_avoidability_dicts(
            maneuver_challenge_dicts,
            trajectory_dicts,
            obs_dicts,
            ctx_dicts,
        )

        ctx_dicts = remove_collision_avoidance_command_using_avoidability(
            obs_dicts, trajectory_dicts, ctx_dicts
        )

        # update the ndd probability according to collision avoidability
        (
            modified_ndd_control_command_dicts,
            ctx_dicts,
        ) = modify_ndd_dict_according_to_avoidability(
            self.unavoidable_collision_prob_factor, maneuver_challenge_dicts, ctx_dicts
        )
        ctx_dicts = update_ndd_distribution_to_vehicle_ctx(
            ctx_dicts, modified_ndd_control_command_dicts
        )

        criticality_dicts, ctx_dicts = get_criticality_dicts(
            maneuver_challenge_dicts, ctx_dicts
        )

        # get the NDD distribution for each vehicle (after the collision avoidance command is added and the ndd probability is adjusted)
        ndd_control_command_dicts = {
            AgentType.VEHICLE: get_ndd_distribution_from_ctx(
                ctx_dicts, AgentType.VEHICLE
            ),
            AgentType.VULNERABLE_ROAD_USER: get_ndd_distribution_from_ctx(
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
                            negligence_hook(agent_id)
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