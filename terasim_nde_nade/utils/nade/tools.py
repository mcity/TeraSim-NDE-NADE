from addict import Dict
from loguru import logger

from terasim.params import AgentType

from ..base import CommandType


def get_ndd_distribution_from_ctx(ctx_dicts, agent_type):
    ndd_control_command_dicts = Dict(
        {
            agent_id: ctx_dicts[agent_type][agent_id]["ndd_command_distribution"]
            for agent_id in ctx_dicts[agent_type]
        }
    )
    return ndd_control_command_dicts

def update_ndd_distribution_to_vehicle_ctx(ctx_dicts, ndd_control_command_dicts):
    for veh_id in ndd_control_command_dicts:
        ctx_dicts[AgentType.VEHICLE][veh_id][
            "ndd_command_distribution"
        ] = ndd_control_command_dicts[veh_id]
    return ctx_dicts

def get_criticality_dicts(maneuver_challenge_dicts, ctx_dicts):
    ndd_control_command_dicts = get_ndd_distribution_from_ctx(
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

def update_control_cmds_from_predicted_trajectory(
    ITE_control_cmds, trajectory_dicts
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
                == "adversarial"
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