import os

import numpy as np
import terasim.utils as utils
from addict import Dict
from loguru import logger
from terasim.overlay import traci
from terasim.params import AgentType
from terasim_nde_nade.utils import (
    CommandType,
    NDECommand,
    predict_future_trajectory,
)
from terasim_nde_nade.envs.utils.maneuver_challenge import get_maneuver_challenge
from terasim_nde_nade.envs.utils.trajectory_prediction import get_ndd_distribution_from_ctx


def unavoidable_maneuver_challenge_hook(veh_id):
    traci.vehicle.highlight(veh_id, (128, 128, 128, 255), duration=0.1)

def negligence_hook(veh_id):
    traci.vehicle.highlight(veh_id, (255, 0, 0, 255), duration=2)

def get_vehicle_avoidance_command(
    neglecting_vehicle_future,
    neglected_vehicle_future,
    neglecting_vehicle_id,
    neglected_vehicle_id,
    emergency_brake_deceleration
):
    avoidance_command = NDECommand(
        command_type=CommandType.ACCELERATION,
        acceleration=-emergency_brake_deceleration,
        prob=0,
        duration=3,
    )
    avoidance_command.info.update(
        {
            "mode": "avoid_collision",
            "neglecting_vehicle_id": neglecting_vehicle_id,
            "neglected_vehicle_id": neglected_vehicle_id,
        }
    )
    return avoidance_command

def get_accept_collision_command():
    accept_command = NDECommand(
        command_type=CommandType.ACCELERATION,
        acceleration=0,
        prob=0,
        duration=2,
    )
    accept_command.info = {"mode": "accept_collision"}
    return accept_command

def get_negligence_pair_dict(ctx_dicts, potential=False):
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

def remove_collision_avoidance_command_using_avoidability(
    obs_dicts, trajectory_dicts, ctx_dicts
):
    """Remove the collision avoidance command for the vehicles that are not avoidable.

    Args:
        obs_dicts (dict): the observation dicts
        trajectory_dicts (dict): the trajectory dicts
        veh_ctx_dicts (dict): the vehicle context dicts

    Returns:
        veh_ctx_dicts (dict): the updated vehicle context dicts
    """
    potential_negligence_pair_dict = get_negligence_pair_dict(
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

def add_avoid_accept_collision_command(
    sumo_net, obs_dicts, trajectory_dicts, ctx_dicts
):
    potential_negligence_pair_dict = get_negligence_pair_dict(
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
            avoidance_command = get_vehicle_avoidance_command(
                neglecting_vehicle_future,
                neglected_vehicle_future,
                neglecting_vehicle_id,
                neglected_vehicle_id,
                traci.vehicle.getEmergencyDecel(neglected_vehicle_id)
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
                sumo_net,
                time_horizon_step=5,
                time_resolution=0.5,
                interpolate_resolution=0.5,
                current_time=None,
                veh_info=None,
            )

            accept_command = get_accept_collision_command()
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
                sumo_net,
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
            avoidance_command = get_vehicle_avoidance_command(
                neglecting_vru_future,
                neglected_vehicle_future,
                neglecting_vru_id,
                neglected_vehicle_id,
                traci.vehicle.getEmergencyDecel(neglected_vehicle_id)
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
                sumo_net,
                time_horizon_step=5,
                time_resolution=0.5,
                interpolate_resolution=0.5,
                current_time=None,
                veh_info=None,
            )

            accept_command = get_accept_collision_command()
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
                sumo_net,
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

def get_avoidability_dicts(
    maneuver_challenge_dicts, trajectory_dicts, obs_dicts, ctx_dicts
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
            ] = get_maneuver_challenge(
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
            ] = get_maneuver_challenge(
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

def modify_ndd_dict_according_to_avoidability(
    unavoidable_collision_prob_factor, maneuver_challenge_dicts, ctx_dicts
):
    ndd_control_command_dicts = get_ndd_distribution_from_ctx(
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
                    * unavoidable_collision_prob_factor
                )
                ndd_control_command_dicts[veh_id]["normal"].prob = (
                    1 - ndd_control_command_dicts[veh_id]["negligence"].prob
                )
                logger.trace(
                    f"{veh_id} is marked as unavoidable collision and the prob is reduced to {ndd_control_command_dicts[veh_id]['negligence'].prob}"
                )
                unavoidable_maneuver_challenge_hook(veh_id)
    return ndd_control_command_dicts, ctx_dicts

def record_negligence_related_information(negligence_pair_dict, ctx_dicts, record):
    if len(negligence_pair_dict[AgentType.VEHICLE]):
        record.event_info[
            utils.get_time()
        ].negligence_pair_dict = negligence_pair_dict[AgentType.VEHICLE]
        record.event_info[utils.get_time()].neglecting_vehicle_id = list(
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

        record.event_info[utils.get_time()].negligence_info_dict = {
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

        record.event_info[
            utils.get_time()
        ].negligence_command = negligence_command_dict
        record.event_info[
            utils.get_time()
        ].neglected_command = neglected_command_dict
    return ctx_dicts

def apply_collision_avoidance(
    trajectory_dicts,
    ctx_dicts,
    ITE_control_command_dict,
    record
):
    """after the NADE decision, apply collision avoidance for the neglected vehicles.

    Args:
        ctx_dicts (_type_): _description_
        ITE_control_command_dict (_type_): _description_

    Returns:
        _type_: _description_
    """

    negligence_pair_dict = get_negligence_pair_dict(ctx_dicts)
    avoid_collision_IS_prob = float(os.getenv("AVOID_COLLISION_IS_PROB", 0.2))
    avoid_collision_ndd_prob = 0.99
    weight = 1.0
    ctx_dicts = record_negligence_related_information(
        negligence_pair_dict, ctx_dicts, record
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
            record.event_info[utils.get_time()].update(
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
                    record.event_info[utils.get_time()].update(
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
                record.event_info[utils.get_time()].update(
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

    record.event_info[utils.get_time()].neglected_command = {
        str(ITE_control_command_dict[neglected_vehicle_id])
        for neglected_vehicle_id in neglected_vehicle_set
    }

    return ITE_control_command_dict, ctx_dicts, weight

