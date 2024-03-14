from terasim_nde_nade.envs.safetest_nde import SafeTestNDE
import sumolib
from terasim.overlay import traci
import terasim.utils as utils
import numpy as np
import math
import os
from terasim.utils import (
    sumo_coordinate_to_center_coordinate,
    sumo_heading_to_orientation,
)
from terasim_nde_nade.vehicle.nde_vehicle_utils import (
    get_collision_type_and_prob,
    Command,
    NDECommand,
    TrajectoryPoint,
    predict_future_trajectory,
    get_vehicle_info,
)
from shapely.geometry import LineString
from terasim_nde_nade.vehicle.nde_vehicle_utils import collision_check, is_intersect
from loguru import logger

veh_length = 5.0
veh_width = 1.85
circle_r = 1.3
tem_len = math.sqrt(circle_r**2 - (veh_width / 2) ** 2)


class Point:
    def __init__(self, position_tuple):
        x, y = position_tuple[0], position_tuple[1]
        self.x = x
        self.y = y

    def __str__(self) -> str:
        return f"({self.x}, {self.y})"


def check_func(PointList):
    for i in range(len(PointList) - 1):
        if (PointList[i].x - PointList[i + 1].x) ** 2 + (
            PointList[i].y - PointList[i + 1].y
        ) ** 2 > 256:
            return False
    return True


import numpy as np


class SafeTestNADE(SafeTestNDE):

    def on_start(self, ctx):
        self.importance_sampling_weight = 1.0
        self.max_importance_sampling_prob = 5e-2
        self.unavoidable_collision_prob_factor = (
            1e-2  # the factor to reduce the probability of the anavoidable collision
        )
        self.early_termination_weight_threshold = 1e-5
        return super().on_start(ctx)

    def executemove(self, ctx, control_cmds, veh_ctx_dicts, obs_dicts):
        traci.simulation.executeMove()
        self._maintain_all_vehicles(ctx)
        existing_vehicle_list = traci.vehicle.getIDList()
        control_cmds = {
            veh_id: control_cmds[veh_id]
            for veh_id in control_cmds
            if veh_id in existing_vehicle_list
        }
        obs_dicts = {
            veh_id: obs_dicts[veh_id]
            for veh_id in obs_dicts
            if veh_id in existing_vehicle_list
        }
        veh_ctx_dicts = {
            veh_id: veh_ctx_dicts[veh_id]
            for veh_id in veh_ctx_dicts
            if veh_id in existing_vehicle_list
        }
        return control_cmds, veh_ctx_dicts, obs_dicts


    # @profile
    def on_step(self, ctx):
        # clear vehicle context dicts
        veh_ctx_dicts = {}
        # Make NDE decisions for all vehicles
        control_cmds, veh_ctx_dicts = self.make_decisions(ctx)
        obs_dicts = self.get_observation_dicts()
        # Make ITE decision, includes the modification of NDD distribution according to avoidability
        control_cmds, veh_ctx_dicts, obs_dicts = self.executemove(ctx, control_cmds, veh_ctx_dicts, obs_dicts)
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

        ITE_control_cmds, veh_ctx_dicts, weight = self.apply_collision_avoidance(
            trajectory_dicts, veh_ctx_dicts, ITE_control_cmds
        )

        ITE_control_cmds = self.update_control_cmds_from_predicted_trajectory(
            ITE_control_cmds, trajectory_dicts
        )

        # record the negligence mode
        # self.negligence_record(ITE_control_cmds)
        # self.monitor.update_vehicle_mode(ITE_control_cmds, self.importance_sampling_weight)
        # # monitor the environment
        # self.monitor.add_observation(ITE_control_cmds, obs_dicts)
        self.refresh_control_commands_state()
        self.execute_control_commands(ITE_control_cmds)
        self.importance_sampling_weight *= (
            weight  # update the importance sampling weight
        )
        # Simulation stop check
        return self.should_continue_simulation()

    def get_observation_dicts(self):
        obs_dicts = {
            vehicle.id: vehicle.observation for vehicle in self.vehicle_list.values()
        }
        return obs_dicts

    def get_time_to_collision(self, distance, speed):
        if distance <= 0:
            ttc = 0
        elif speed <= 0:
            ttc = 100
        else:
            ttc = distance / speed
        return ttc

    # @profile
    def NADE_decision(self, control_command_dicts, veh_ctx_dicts, obs_dicts):
        """NADE decision here.

        Args:
            control_command_dicts
            obs_dicts
        """
        trajectory_dicts, veh_ctx_dicts = self.predict_future_trajectory_dicts(
            obs_dicts, veh_ctx_dicts
        )  # predict future trajectories for each vehicle (normal/negligence)

        # get maneuver challenge and collision avoidability
        # will mark the conflict vehicles in the veh_ctx_dicts
        maneuver_challenge_dicts, veh_ctx_dicts = self.get_maneuver_challenge_dicts(
            trajectory_dicts,
            obs_dicts,
            veh_ctx_dicts,
        )

        # add collision avoidance command for the neglected vehicles, and predict the future trajectories for the avoidance command using the veh_ctx_dicts
        trajectory_dicts, veh_ctx_dicts = self.add_collision_avoidance_command(
            obs_dicts, trajectory_dicts, veh_ctx_dicts
        )

        # update the ndd probability according to collision avoidability
        avoidability_dicts, veh_ctx_dicts = self.get_avoidability_dicts(
            maneuver_challenge_dicts,
            trajectory_dicts,
            obs_dicts,
            veh_ctx_dicts,
        )

        # update the ndd probability according to collision avoidability
        modified_ndd_control_command_dicts, veh_ctx_dicts = (
            self.modify_ndd_dict_according_to_avoidability(
                maneuver_challenge_dicts, veh_ctx_dicts
            )
        )

        # highlight the critical vehicles and calculate criticality
        self.highlight_critical_vehicles(maneuver_challenge_dicts, veh_ctx_dicts)

        criticality_dicts, veh_ctx_dicts = self.get_criticality_dicts(
            maneuver_challenge_dicts, veh_ctx_dicts
        )

        # get the NDD distribution for each vehicle (after the collision avoidance command is added and the ndd probability is adjusted)
        ndd_control_command_dicts = self.get_ndd_distribution_from_vehicle_ctx(
            veh_ctx_dicts
        )
        ITE_control_command_dicts, veh_ctx_dicts, weight = (
            self.NADE_importance_sampling(
                ndd_control_command_dicts, maneuver_challenge_dicts, veh_ctx_dicts
            )
        )

        return (
            ITE_control_command_dicts,
            veh_ctx_dicts,
            weight,
            trajectory_dicts,
            maneuver_challenge_dicts,
            criticality_dicts,
        )

    def highlight_critical_vehicles(self, maneuver_challenge_dicts, veh_ctx_dicts):
        for veh_id in maneuver_challenge_dicts:
            if (
                veh_ctx_dicts[veh_id].get("avoidable", True) is False
            ):  # collision unavoidable
                # highlight the vehicle with gray
                traci.vehicle.highlight(veh_id, (128, 128, 128, 255), duration=0.1)
            elif maneuver_challenge_dicts[veh_id].get(
                "negligence"
            ):  # avoidable collision
                # highlight the vehicle with red
                traci.vehicle.highlight(veh_id, (255, 0, 0, 255), duration=0.1)

    def get_ndd_distribution_from_vehicle_ctx(self, veh_ctx_dicts):
        if len(veh_ctx_dicts) == 1:
            return veh_ctx_dicts[list(veh_ctx_dicts.keys())[0]][
                "ndd_command_distribution"
            ]
        ndd_control_command_dicts = {
            veh_id: veh_ctx_dicts[veh_id]["ndd_command_distribution"]
            for veh_id in veh_ctx_dicts
        }
        return ndd_control_command_dicts

    # @profile
    def predict_future_trajectory_dicts(self, obs_dicts, veh_ctx_dicts):
        # predict future trajectories for each vehicle
        sumo_net = self.simulator.sumo_net
        current_time = traci.simulation.getTime()
        trajectory_dicts = {}
        ndd_control_command_dicts = self.get_ndd_distribution_from_vehicle_ctx(
            veh_ctx_dicts
        )
        for veh_id in ndd_control_command_dicts:
            obs_dict = obs_dicts[veh_id]
            veh_info = get_vehicle_info(veh_id, obs_dict, sumo_net)
            control_command_dict = ndd_control_command_dicts[veh_id]

            trajectory_dict = {
                modality: predict_future_trajectory(
                    veh_id,
                    obs_dict,
                    control_command_dict[modality],
                    sumo_net,
                    time_horizon_step=6,
                    time_resolution=0.5,
                    interpolate_resolution=0.5,
                    current_time=current_time,
                    veh_info=veh_info,
                )
                for modality in control_command_dict
            }
            trajectory_dicts[veh_id] = trajectory_dict
        return trajectory_dicts, veh_ctx_dicts

    def update_control_cmds_from_predicted_trajectory(
        self, ITE_control_cmds, trajectory_dicts
    ):
        """Update the veh_ctx_dicts with the predicted future trajectories.
        change the command type from acceleration to trajectory if the predicted collision type is rearend
        """
        for veh_id in ITE_control_cmds:
            if (
                ITE_control_cmds[veh_id].info.get("mode") == "avoid_collision"
                or ITE_control_cmds[veh_id].info.get("mode") == "negligence"
            ):
                if ITE_control_cmds[veh_id].command_type == Command.ACCELERATION:
                    ITE_control_cmds[veh_id].command_type = Command.TRAJECTORY
                    ITE_control_cmds[veh_id].future_trajectory = trajectory_dicts[veh_id][
                        ITE_control_cmds[veh_id].info.get("mode")
                    ]
                    logger.trace(
                        f"veh_id: {veh_id} is updated to trajectory command with mode: {ITE_control_cmds[veh_id].info.get('mode')}"
                    )
        return ITE_control_cmds

    def modify_ndd_dict_according_to_avoidability(
        self, maneuver_challenge_dicts, veh_ctx_dicts
    ):
        ndd_control_command_dicts = self.get_ndd_distribution_from_vehicle_ctx(
            veh_ctx_dicts
        )

        for veh_id in maneuver_challenge_dicts:
            # if the vehicle negligence control command do has the potential to collide with other vehicles
            if maneuver_challenge_dicts[veh_id].get("negligence"):
                # mark all rearend collision as unavoidable
                if veh_ctx_dicts[veh_id].get("avoidable", True) is False:
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
        return ndd_control_command_dicts, veh_ctx_dicts

    def get_negligence_pair_dict(self, veh_ctx_dicts, potential=False):
        """Get the negligence pair dict.
        potential: if True, return the potential negligence pair dict, otherwise return the negligence pair dict (vehicle actually do negligence).
        """
        veh_ctx_dicts = {
            veh_id: ctx
            for veh_id, ctx in veh_ctx_dicts.items()
            if "conflict_vehicle" in ctx
        }

        if potential:
            negligence_pair_dict = {
                veh_id: ctx["conflict_vehicle"] for veh_id, ctx in veh_ctx_dicts.items()
            }
        else:
            negligence_pair_dict = {
                veh_id: ctx["conflict_vehicle"]
                for veh_id, ctx in veh_ctx_dicts.items()
                if "mode" in ctx and ctx["mode"] == "negligence"
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
            command_type=Command.ACCELERATION,
            acceleration=-emergency_brake_deceleration,
            prob=0,
            duration=2,
        )  # by default, this avoidance command prob is 0, only when really being neglected, the prob will be updated
        avoidance_command.info.update(
            {
                "mode": "avoid_collision",
                "neglecting_vehicle_id": neglecting_vehicle_id,
                "neglected_vehicle_id": neglected_vehicle_id,
            }
        )
        return avoidance_command

    def add_collision_avoidance_command(
        self, obs_dicts, trajectory_dicts, veh_ctx_dicts
    ):
        potential_negligence_pair_dict = self.get_negligence_pair_dict(
            veh_ctx_dicts, potential=True
        )
        for (
            neglecting_vehicle_id,
            neglected_vehicle_list,
        ) in potential_negligence_pair_dict.items():
            neglecting_vehicle_future = trajectory_dicts[neglecting_vehicle_id][
                "negligence"
            ]
            for neglected_vehicle_id in neglected_vehicle_list:
                neglected_vehicle_future = trajectory_dicts[neglected_vehicle_id][
                    "normal"
                ]
                avoidance_command = self.get_vehicle_avoidance_command(
                    neglecting_vehicle_future,
                    neglected_vehicle_future,
                    neglecting_vehicle_id,
                    neglected_vehicle_id,
                )
                veh_ctx_dicts[neglected_vehicle_id]["ndd_command_distribution"][
                    "avoid_collision"
                ] = avoidance_command
                trajectory_dicts[neglected_vehicle_id]["avoid_collision"] = (
                    predict_future_trajectory(
                        neglected_vehicle_id,
                        obs_dicts[neglected_vehicle_id],
                        avoidance_command,
                        self.simulator.sumo_net,
                        time_horizon_step=6,
                        time_resolution=0.5,
                        interpolate_resolution=0.5,
                        current_time=None,
                        veh_info=None,
                    )
                )
            logger.trace(
                f"add avoidance command for vehicle: {neglected_vehicle_list} from vehicle: {neglecting_vehicle_id}"
            )
        return trajectory_dicts, veh_ctx_dicts

    def apply_collision_avoidance(
        self, trajectory_dicts, veh_ctx_dicts, ITE_control_command_dict
    ):
        """after the NADE decision, apply collision avoidance for the neglected vehicles.

        Args:
            veh_ctx_dicts (_type_): _description_
            ITE_control_command_dict (_type_): _description_

        Returns:
            _type_: _description_
        """
        negligence_pair_dict = self.get_negligence_pair_dict(veh_ctx_dicts)

        avoid_collision_IS_prob = float(os.getenv("AVOID_COLLISION_IS_PROB", 0.2))
        avoid_collision_IS_prob = 1.0
        avoid_collision_ndd_prob = 0.99
        weight = 1.0

        # no vehicle neglected
        if len(negligence_pair_dict) == 0:
            return ITE_control_command_dict, veh_ctx_dicts, weight

        timestamp = utils.get_time()
        IS_prob = np.random.uniform(0, 1)
        if IS_prob < avoid_collision_IS_prob:  # apply collision aboidance (select NDD)
            for (
                neglecting_vehicle_id,
                neglected_vehicle_list,
            ) in negligence_pair_dict.items():
                logger.info(
                    f"{timestamp}, neglected_vehicle_list: {neglected_vehicle_list} avoiding collision from {neglecting_vehicle_id}, avoidability: {veh_ctx_dicts[neglecting_vehicle_id
                ].get("avoidable", True)}"
                )
                for neglected_vehicle_id in neglected_vehicle_list:
                    avoid_collision_command = veh_ctx_dicts[neglected_vehicle_id][
                        "ndd_command_distribution"
                    ]["avoid_collision"]
                    ITE_control_command_dict[neglected_vehicle_id] = (
                        avoid_collision_command
                    )
        else:  # does not apply collision avoidance
            for (
                neglecting_vehicle_id,
                neglected_vehicle_list,
            ) in negligence_pair_dict.items():
                logger.info(
                    f"{timestamp}, neglected_vehicle_list: {neglected_vehicle_list} accept collision from {neglecting_vehicle_id}, avoidability: {veh_ctx_dicts[
                    neglecting_vehicle_id
                ].get("avoidable", True)}"
                )
                for neglected_vehicle_id in neglected_vehicle_list:
                    veh_ctx_dicts[neglected_vehicle_id]["mode"] = "accept_collision"
            weight *= (1 - avoid_collision_ndd_prob) / (1 - avoid_collision_IS_prob)
        return ITE_control_command_dict, veh_ctx_dicts, weight

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
        self, ndd_control_command_dicts, maneuver_challenge_dicts, veh_ctx_dicts
    ):
        """Importance sampling for NADE.

        Args:
            ndd_control_command_dict (dict): for each vehicle v_id, ndd_control_command_dict[veh_id] = ndd_control_command, ndd_pdf
            criticality_dict (dict): for each vehicle v_id, criticality_dict[veh_id] = criticality

        Returns:
            weight (float): the importance sampling weight
        """
        weight = 1.0
        # intialize the ITE control command dict with the same keys as the ndd_control_command_dict
        ITE_control_command_dict = {
            veh_id: ndd_control_command_dicts[veh_id]["normal"]
            for veh_id in ndd_control_command_dicts
        }

        for veh_id in maneuver_challenge_dicts:
            if maneuver_challenge_dicts[veh_id].get("negligence"):
                ndd_normal_prob = ndd_control_command_dicts[veh_id]["normal"].prob
                ndd_negligence_prob = ndd_control_command_dicts[veh_id][
                    "negligence"
                ].prob
                assert (
                    ndd_normal_prob + ndd_negligence_prob == 1
                ), "The sum of the probabilities of the normal and negligence control commands should be 1."

                # get the importance sampling probability
                IS_prob = self.get_IS_prob(
                    veh_id,
                    ndd_control_command_dicts,
                    maneuver_challenge_dicts,
                    veh_ctx_dicts,
                )

                # update the importance sampling weight and the ITE control command
                sampled_prob = np.random.uniform(0, 1)
                if sampled_prob < IS_prob:  # select the negligece control command
                    weight *= ndd_negligence_prob / IS_prob
                    ITE_control_command_dict[veh_id] = ndd_control_command_dicts[
                        veh_id
                    ]["negligence"]
                    veh_ctx_dicts[veh_id]["mode"] = "negligence"
                    logger.info(
                        f"time: {utils.get_time()}, veh_id: {veh_id} select negligence control command, IS_prob: {IS_prob}, weight: {self.importance_sampling_weight}"
                    )
                else:
                    weight *= ndd_normal_prob / (1 - IS_prob)
                    ITE_control_command_dict[veh_id] = ndd_control_command_dicts[
                        veh_id
                    ]["normal"]
                    logger.trace(
                        f"time: {utils.get_time()}, veh_id: {veh_id} select normal control command, IS_prob: {IS_prob}, weight: {self.importance_sampling_weight}"
                    )
        return ITE_control_command_dict, veh_ctx_dicts, weight

    def get_IS_prob(
        self, veh_id, ndd_control_command_dicts, maneuver_challenge_dicts, veh_ctx_dicts
    ):
        if maneuver_challenge_dicts[veh_id].get("negligence"):
            if veh_ctx_dicts[veh_id].get("avoidable", True) is False:
                IS_magnitude = 0.1
            IS_magnitude = float(os.getenv("IS_MAGNITUDE_INTERSECTION", 100))
            try:
                predicted_collision_type = ndd_control_command_dicts[veh_id][
                    "negligence"
                ].info["predicted_collision_type"]

                # get the importance sampling magnitude according to the predicted collision type
                if "roundabout" in predicted_collision_type:
                    IS_magnitude = float(os.getenv("IS_MAGNITUDE_ROUNDABOUT", 100))
                    logger.trace(f"IS_magnitude: {IS_magnitude} for roundabout")
                elif "highway" in predicted_collision_type:
                    IS_magnitude = float(os.getenv("IS_MAGNITUDE_HIGHWAY", 100))
                    logger.trace(f"IS_magnitude: {IS_magnitude} for highway")
                else:
                    IS_magnitude = float(os.getenv("IS_MAGNITUDE_INTERSECTION", 1000))
                    logger.trace(f"IS_magnitude: {IS_magnitude} for intersection")

                # if the vehicle is not avoidable, increase the importance sampling magnitude
                if not veh_ctx_dicts[veh_id].get("avoidable", True):
                    IS_magnitude = IS_magnitude * 10
            except Exception as e:
                logger.critical(
                    f"Error in getting the importance sampling magnitude: {e}"
                )
                pass
            return np.clip(
                ndd_control_command_dicts[veh_id]["negligence"].prob * IS_magnitude,
                0,
                self.max_importance_sampling_prob,
            )
        else:
            raise Exception("The vehicle is not in the negligence mode.")

    def get_avoidability_dicts(
        self, maneuver_challenge_dicts, trajectory_dicts, obs_dicts, veh_ctx_dicts
    ):
        negligence_future_trajectory_dict = {
            veh_id: trajectory_dicts[veh_id].get("negligence", None)
            for veh_id in trajectory_dicts
        }
        avoidance_future_trajectory_dict = {
            veh_id: trajectory_dicts[veh_id].get("avoid_collision", None)
            for veh_id in trajectory_dicts
        }

        # initialize the avoidability of each vehicle
        for veh_id in maneuver_challenge_dicts:
            veh_ctx_dicts[veh_id]["avoidable"] = True

        # get the maneuver challenge for the negligence vehicle future and the avoidance vehicle future
        maneuver_challenge_avoidance_dicts = {}
        for veh_id in maneuver_challenge_dicts:
            if maneuver_challenge_dicts[veh_id].get("negligence"):
                conflict_vehicle_list = veh_ctx_dicts[veh_id].get(
                    "conflict_vehicle", []
                )
                conflict_vehicle_future_dict = {
                    veh_id: avoidance_future_trajectory_dict[veh_id]
                    for veh_id in conflict_vehicle_list
                }
                maneuver_challenge_avoidance_dicts[veh_id] = (
                    self.get_maneuver_challenge(
                        veh_id,
                        negligence_future_trajectory_dict[veh_id],
                        conflict_vehicle_future_dict,
                        obs_dicts,
                        veh_ctx_dicts[veh_id],
                        record_in_ctx=False,
                    )
                )
                if maneuver_challenge_avoidance_dicts[veh_id].get("negligence"):
                    veh_ctx_dicts[veh_id]["avoidable"] = False
                    logger.trace(f"veh_id: {veh_id} is not avoidable")

        return maneuver_challenge_avoidance_dicts, veh_ctx_dicts

    # @profile
    def get_maneuver_challenge_dicts(self, trajectory_dicts, obs_dicts, veh_ctx_dicts):
        """Get the maneuver challenge for each vehicle when it is in the negligence mode while other vehicles are in the normal mode.

        Args:
            trajectory_dicts (dict): trajectory_dicts[veh_id] = {"normal": normal_future_trajectory, "negligence": negligence_future_trajectory}

        Returns:
            maneuver_challenge_dict (dict): maneuver_challenge_dict[veh_id] = (num_affected_vehicles, affected_vehicles)
        """
        normal_future_trajectory_dict = {
            veh_id: trajectory_dicts[veh_id].get("normal", None)
            for veh_id in trajectory_dicts
        }
        negligence_future_trajectory_dict = {
            veh_id: trajectory_dicts[veh_id].get("negligence", None)
            for veh_id in trajectory_dicts
        }

        # get the maneuver challenge for each vehicle, check if the negligence future will collide with other vehicles' normal future
        maneuver_challenge_dicts = {
            veh_id: self.get_maneuver_challenge(
                veh_id,
                negligence_future_trajectory_dict[veh_id],
                normal_future_trajectory_dict,
                obs_dicts,
                veh_ctx_dicts[veh_id],
                record_in_ctx=True,
            )
            for veh_id in trajectory_dicts
        }

        for veh_id in veh_ctx_dicts:
            veh_ctx_dicts[veh_id]["maneuver_challenge"] = (
                maneuver_challenge_dicts[veh_id]
                if veh_id in maneuver_challenge_dicts
                else {"normal": 0}
            )

        maneuver_challenge_dicts_shrinked = {
            veh_id: maneuver_challenge_dicts[veh_id]
            for veh_id in maneuver_challenge_dicts
            if maneuver_challenge_dicts[veh_id].get("negligence")
        }
        conflict_vehicle_info = {
            veh_id: veh_ctx_dicts[veh_id].get("conflict_vehicle")
            for veh_id in veh_ctx_dicts
            if veh_ctx_dicts[veh_id].get("conflict_vehicle")
        }
        logger.trace(
            f"maneuver_challenge: {maneuver_challenge_dicts_shrinked}, conflict_vehicle_info: {conflict_vehicle_info}"
        )
        return maneuver_challenge_dicts, veh_ctx_dicts

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
        """Get the maneuver challenge for the negligence vehicle.

        Args:
            negligence_veh_id (str): the id of the negligence vehicle
            negligence_veh_future (list): the future trajectory of the negligence vehicle
            all_normal_veh_future (dict): all_normal_veh _future[veh_id] = future trajectory of the normal vehicle

        Returns:
            num_affected_vehicles (int): the number of vehicles that will be affected by the negligence vehicle
            maneuver_challenge_info (dict): maneuver_challenge_info[veh_id] = 1 if the negligence vehicle will affect the normal vehicle
        """
        # see if the one negligence future will intersect with other normal futures
        final_collision_flag = False
        if negligence_veh_future is not None and all_normal_veh_future is not None:
            for veh_id in all_normal_veh_future:
                if veh_id == negligence_veh_id:
                    continue
                if all_normal_veh_future[veh_id] is None:
                    print(
                        f"veh_id: {veh_id}, all_normal_veh_future[veh_id]: {all_normal_veh_future[veh_id]}"
                    )
                link_intersection_flag = is_link_intersect(
                    obs_dicts[negligence_veh_id], obs_dicts[veh_id]
                )
                if not link_intersection_flag:
                    continue  # if the next link of the two vehicles are not intersected, then the two vehicles will not collide
                collision_flag = is_intersect(
                    negligence_veh_future,
                    all_normal_veh_future[veh_id],
                    veh_length,
                    tem_len,
                    circle_r,
                )
                final_collision_flag = final_collision_flag or collision_flag
                if collision_flag and record_in_ctx:
                    if "conflict_vehicle" not in veh_ctx_dict:
                        veh_ctx_dict["conflict_vehicle"] = []
                    veh_ctx_dict["conflict_vehicle"].append(veh_id)
            return {
                "normal": 0,
                "negligence": (1 if final_collision_flag else 0),
            }  # the first element is the number of vehicles that will be affected by the negligence vehicle
        else:
            return {"normal": 0}

    def get_criticality_dicts(self, maneuver_challenge_dicts, veh_ctx_dicts):
        ndd_control_command_dicts = self.get_ndd_distribution_from_vehicle_ctx(
            veh_ctx_dicts
        )
        criticality_dicts = {}
        for veh_id in maneuver_challenge_dicts:
            ndd_control_command_dict = ndd_control_command_dicts[veh_id]
            maneuver_challenge_dict = maneuver_challenge_dicts[veh_id]
            criticality_dicts[veh_id] = {
                modality: ndd_control_command_dict[modality].prob
                * maneuver_challenge_dict[modality]
                for modality in maneuver_challenge_dict
                if modality != "info"
            }
        for veh_id in veh_ctx_dicts:
            veh_ctx_dicts[veh_id]["criticality"] = (
                criticality_dicts[veh_id]
                if veh_id in criticality_dicts
                else {"normal": 0}
            )
        return criticality_dicts, veh_ctx_dicts


def is_link_intersect(veh1_obs, veh2_obs):
    veh_1_edge_id = veh1_obs["ego"]["edge_id"]
    veh_2_edge_id = veh2_obs["ego"]["edge_id"]
    if veh_1_edge_id == veh_2_edge_id:
        return True

    veh1_next_lane_id_set = set(veh1_obs["ego"]["upcoming_lanes"])
    veh2_next_lane_id_set = set(veh2_obs["ego"]["upcoming_lanes"])

    if veh1_next_lane_id_set.intersection(veh2_next_lane_id_set):
        return True

    veh1_foe_lane_id_set = veh1_obs["ego"]["upcoming_foe_lane_id_set"]
    veh2_foe_lane_id_set = veh2_obs["ego"]["upcoming_foe_lane_id_set"]

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
