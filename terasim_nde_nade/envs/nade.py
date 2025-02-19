import json
import math
import os
import numpy as np
from addict import Dict
from loguru import logger

from terasim.overlay import profile, traci
from terasim.params import AgentType
import terasim.utils as utils
from terasim_nde_nade.envs.nde import NDE
from terasim_nde_nade.utils import (
    CommandType, 
    predict_environment_future_trajectory, 
    get_environment_maneuver_challenge,
    add_avoid_accept_collision_command,
    get_environment_avoidability,
    modify_nde_cmd_veh_using_avoidability,
    remove_collision_avoidance_command_using_avoidability,
    apply_collision_avoidance,
    get_environment_criticality,
    get_nde_cmd_from_cmd_info,
    update_nde_cmd_to_vehicle_cmd_info,
    update_control_cmds_from_predicted_trajectory,
    adversarial_hook,
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
        # Step 1. Make NDE decisions for all vehicles and vrus
        _, env_command_information = super().make_decisions(ctx)
        env_observation = self.get_env_observation()
        (
            env_command_information,
            env_observation,
            should_continue_simulation_flag,
        ) = self.executeMove(ctx, env_command_information, env_observation)
        # Step 2. Make ITE decision, includes the modification of NDD distribution according to avoidability
        # if should_continue_simulation_flag:
        (
            nade_control_commands,
            env_command_information,
            weight,
            env_future_trajectory,
            _,
            _,
        ) = self.NADE_decision(
            env_command_information, env_observation
        )  # enable ITE
        self.importance_sampling_weight *= weight  # update weight by adversarial
        nade_control_commands, env_command_information, weight = apply_collision_avoidance(
            env_future_trajectory, env_command_information, nade_control_commands, self.record
        )
        self.importance_sampling_weight *= (
            weight  # update weight by collision avoidance
        )
        nade_control_commands = update_control_cmds_from_predicted_trajectory(
            nade_control_commands, env_future_trajectory
        )
        if hasattr(self, "nnde_make_decisions"):
            nnde_control_commands, _ = self.nnde_make_decisions(ctx)
            nade_control_commands = self.merge_NADE_NeuralNDE_control_commands(
                nade_control_commands, nnde_control_commands
            )
        self.refresh_control_commands_state()
        self.execute_control_commands(nade_control_commands)

        self.record_step_data(env_command_information)
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
            # find if one of veh_1_id or veh_2_id is in the record.event_info adversarial_pair_dict
            for timestep in self.record.event_info:
                adversarial_pair_dict = self.record.event_info[
                    timestep
                ].adversarial_pair_dict
                adversarial_related_vehicle_set = set()
                for adversarial_veh_id in adversarial_pair_dict:
                    adversarial_related_vehicle_set.add(adversarial_veh_id)
                    adversarial_related_vehicle_set.update(
                        adversarial_pair_dict[adversarial_veh_id]
                    )
                if set([veh_1_id, veh_2_id]).issubset(adversarial_related_vehicle_set):
                    self.record.adversarial_event_time = timestep
                    self.record.adversarial_event_info = self.record.event_info[timestep]

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

    def record_step_data(self, env_command_information):
        step_log = Dict()
        for veh_id, veh_command_info in env_command_information[AgentType.VEHICLE].items():
            maneuver_challenge = veh_command_info.get("maneuver_challenge", None)
            if maneuver_challenge and maneuver_challenge.get("adversarial", None):
                step_log[veh_id]["maneuver_challenge"] = maneuver_challenge

            keys = ["avoidable", "conflict_vehicle_list", "mode"]
            step_log[veh_id].update(
                {key: veh_command_info[key] for key in keys if veh_command_info.get(key)}
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
    def NADE_decision(self, env_command_information, env_observation):
        """NADE decision here.

        Args:
            env_command_information
            env_observation
        """
        env_future_trajectory = predict_environment_future_trajectory(
            env_command_information, env_observation, self.simulator.sumo_net
        )
        # get maneuver challenge and collision avoidability
        # will mark the conflict vehicles
        env_maneuver_challenge, env_command_information = get_environment_maneuver_challenge(
            env_future_trajectory,
            env_observation,
            env_command_information,
        )

        # add collision avoidance command for the victim vehicles, and predict the future trajectories for the avoidance command using the veh_ctx_dicts
        env_future_trajectory, env_command_information = add_avoid_accept_collision_command(
            env_future_trajectory,
            env_maneuver_challenge,
            env_observation,
            env_command_information,
            self.simulator.sumo_net,
        )

        # update the ndd probability according to collision avoidability
        env_avoidability, env_command_information = get_environment_avoidability(
            env_maneuver_challenge,
            env_future_trajectory,
            env_observation,
            env_command_information,
        )

        env_command_information = remove_collision_avoidance_command_using_avoidability(
            env_observation, env_future_trajectory, env_command_information
        )

        # update the ndd probability according to collision avoidability
        (
            tmp_nde_control_commands_veh,
            env_command_information,
        ) = modify_nde_cmd_veh_using_avoidability(
            self.unavoidable_collision_prob_factor, env_maneuver_challenge, env_command_information
        )
        env_command_information = update_nde_cmd_to_vehicle_cmd_info(
            env_command_information, tmp_nde_control_commands_veh
        )

        env_criticality, env_command_information = get_environment_criticality(
            env_maneuver_challenge, env_command_information
        )

        # get the NDD distribution for each vehicle (after the collision avoidance command is added and the ndd probability is adjusted)
        nde_control_commands = {
            AgentType.VEHICLE: get_nde_cmd_from_cmd_info(
                env_command_information, AgentType.VEHICLE
            ),
            AgentType.VULNERABLE_ROAD_USER: get_nde_cmd_from_cmd_info(
                env_command_information, AgentType.VULNERABLE_ROAD_USER
            ),
        }
        self.step_epsilon = 1.0
        self.step_weight = 1.0
        if self.allow_NADE_IS:
            (
                nade_control_commands,
                env_command_information,
                weight,
                adversarial_flag,
            ) = self.NADE_importance_sampling(
                nde_control_commands, env_maneuver_challenge, env_command_information
            )
            if adversarial_flag:
                self.latest_IS_time = utils.get_time()
                self.allow_NADE_IS = False
        else:
            weight = 1.0
            nade_control_commands = Dict(
                {
                    AgentType.VEHICLE: {
                        veh_id: nde_control_commands[AgentType.VEHICLE][veh_id][
                            "normal"
                        ]
                        for veh_id in nde_control_commands[AgentType.VEHICLE]
                    },
                    AgentType.VULNERABLE_ROAD_USER: {
                        vru_id: nde_control_commands[
                            AgentType.VULNERABLE_ROAD_USER
                        ][vru_id]["normal"]
                        for vru_id in nde_control_commands[
                            AgentType.VULNERABLE_ROAD_USER
                        ]
                    },
                }
            )
            if utils.get_time() - self.latest_IS_time >= 2.9:
                self.allow_NADE_IS = True

        return (
            nade_control_commands,
            env_command_information,
            weight,
            env_future_trajectory,
            env_maneuver_challenge,
            env_criticality,
        )

    def NADE_importance_sampling(
        self,
        nde_control_commands,
        env_maneuver_challenge,
        env_command_information,
        exclude_IS_agent_set=None,
    ):
        """Importance sampling for NADE.

        Args:
            nde_control_commands (dict): for each vehicle v_id, ndd_control_command_dict[veh_id] = ndd_control_command, ndd_pdf
            criticality_dict (dict): for each vehicle v_id, criticality_dict[veh_id] = criticality

        Returns:
            weight (float): the importance sampling weight
        """
        weight = 1.0
        epsilon = 1.0
        # intialize the ITE control command dict with the same keys as the ndd_control_command_dict
        nade_control_commands = Dict(
            {
                AgentType.VEHICLE: {
                    veh_id: nde_control_commands[AgentType.VEHICLE][veh_id][
                        "normal"
                    ]
                    for veh_id in nde_control_commands[AgentType.VEHICLE]
                },
                AgentType.VULNERABLE_ROAD_USER: {
                    vru_id: nde_control_commands[AgentType.VULNERABLE_ROAD_USER][
                        vru_id
                    ]["normal"]
                    for vru_id in nde_control_commands[
                        AgentType.VULNERABLE_ROAD_USER
                    ]
                },
            }
        )
        adversarial_flag = False
        exclude_IS_agent_set = (
            set() if exclude_IS_agent_set is None else exclude_IS_agent_set
        )

        for agent_type in [AgentType.VEHICLE, AgentType.VULNERABLE_ROAD_USER]:
            for agent_id in env_maneuver_challenge[agent_type]:
                if agent_id in exclude_IS_agent_set:
                    continue
                if env_maneuver_challenge[agent_type][agent_id].get("adversarial"):
                    ndd_normal_prob = nde_control_commands[agent_type][
                        agent_id
                    ].normal.prob
                    ndd_adversarial_prob = nde_control_commands[agent_type][
                        agent_id
                    ].adversarial.prob
                    assert (
                        ndd_normal_prob + ndd_adversarial_prob == 1
                    ), "The sum of the probabilities of the normal and adversarial control commands should be 1."

                    # get the importance sampling probability
                    IS_prob = self.get_IS_prob(
                        agent_id,
                        nde_control_commands[agent_type],
                        env_maneuver_challenge[agent_type],
                        env_command_information[agent_type],
                    )
                    epsilon = 1 - IS_prob

                    # update the importance sampling weight and the ITE control command
                    sampled_prob = np.random.uniform(0, 1)
                    if sampled_prob < IS_prob:  # select the negligece control command
                        weight *= ndd_adversarial_prob / IS_prob
                        nade_control_commands[agent_type][
                            agent_id
                        ] = nde_control_commands[agent_type][agent_id].adversarial
                        env_command_information[agent_type][agent_id]["mode"] = "adversarial"
                        if agent_type == AgentType.VEHICLE:
                            adversarial_hook(agent_id)
                        logger.info(
                            f"time: {utils.get_time()}, agent_id: {agent_id} select adversarial control command, IS_prob: {IS_prob}, ndd_prob: {ndd_adversarial_prob}, weight: {self.importance_sampling_weight}"
                        )
                        adversarial_flag = True
                    else:
                        weight *= ndd_normal_prob / (1 - IS_prob)
                        nade_control_commands[agent_type][
                            agent_id
                        ] = nde_control_commands[agent_type][agent_id]["normal"]
                        logger.trace(
                            f"time: {utils.get_time()}, agent_id: {agent_id} select normal control command, IS_prob: {IS_prob}, weight: {self.importance_sampling_weight}"
                        )
        self.step_epsilon = epsilon
        self.step_weight = weight
        return nade_control_commands, env_command_information, weight, adversarial_flag

    def get_IS_prob(
        self, agent_id, nde_control_commands, env_maneuver_challenge, env_command_information
    ):
        if not env_maneuver_challenge[agent_id].get("adversarial"):
            raise ValueError("The vehicle is not in the adversarial mode.")

        IS_magnitude = IS_MAGNITUDE_DEFAULT
        try:
            predicted_collision_type = nde_control_commands[
                agent_id
            ].adversarial.info["predicted_collision_type"]

            # get the importance sampling magnitude according to the predicted collision type
            for collision_type, env_var in IS_MAGNITUDE_MAPPING.items():
                if collision_type in predicted_collision_type:
                    IS_magnitude = float(os.getenv(env_var, IS_magnitude))
                    break

            # if the vehicle is not avoidable, increase the importance sampling magnitude
            if not env_command_information[agent_id].get("avoidable", True):
                IS_magnitude *= IS_MAGNITUDE_MULTIPLIER
            # logger.trace(f"IS_magnitude: {IS_magnitude} for {collision_type}")
            # logger.trace(f"Original prob: {nde_control_commands[veh_id]["adversarial"].prob}")
            # final_is_prob = np.clip(
            #     nde_control_commands[veh_id]["adversarial"].prob * IS_magnitude,
            #     0,
            #     self.max_importance_sampling_prob,
            # )
            # logger.trace(f"final IS prob for veh_id: {final_is_prob}")

        except Exception as e:
            logger.critical(f"Error in getting the importance sampling magnitude: {e}")

        return np.clip(
            nde_control_commands[agent_id]["adversarial"].prob * IS_magnitude,
            0,
            self.max_importance_sampling_prob,
        )