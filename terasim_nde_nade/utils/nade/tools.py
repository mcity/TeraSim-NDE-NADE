from addict import Dict
from loguru import logger

from terasim.overlay import traci
from terasim.params import AgentType

from ..base import CommandType


def unavoidable_maneuver_challenge_hook(veh_id):
    traci.vehicle.highlight(veh_id, (128, 128, 128, 255), duration=0.1)

def adversarial_hook(veh_id):
    traci.vehicle.highlight(veh_id, (255, 0, 0, 255), duration=2)

def get_nde_cmd_from_cmd_info(env_command_information, agent_type):
    nde_control_commands_agenttype = Dict(
        {
            agent_id: env_command_information[agent_type][agent_id]["ndd_command_distribution"]
            for agent_id in env_command_information[agent_type]
        }
    )
    return nde_control_commands_agenttype

def update_nde_cmd_to_vehicle_cmd_info(env_command_information, nde_control_commands_veh):
    for veh_id in nde_control_commands_veh:
        env_command_information[AgentType.VEHICLE][veh_id][
            "ndd_command_distribution"
        ] = nde_control_commands_veh[veh_id]
    return env_command_information

def get_environment_criticality(env_maneuver_challenge, env_command_information):
    nde_control_commands_veh = get_nde_cmd_from_cmd_info(
        env_command_information, AgentType.VEHICLE
    )
    env_criticality = {}
    for veh_id in env_maneuver_challenge[AgentType.VEHICLE]:
        ndd_control_command_dict = nde_control_commands_veh[veh_id]
        maneuver_challenge_dict = env_maneuver_challenge[AgentType.VEHICLE][
            veh_id
        ]
        env_criticality[veh_id] = {
            modality: ndd_control_command_dict[modality].prob
            * maneuver_challenge_dict[modality]
            for modality in maneuver_challenge_dict
            if modality != "info"
        }
    for veh_id in env_command_information[AgentType.VEHICLE]:
        env_command_information[AgentType.VEHICLE][veh_id]["criticality"] = (
            env_criticality[veh_id]
            if veh_id in env_criticality
            else {"normal": 0}
        )
    return env_criticality, env_command_information

def update_control_cmds_from_predicted_trajectory(
    ITE_control_cmds, env_future_trajectory
):
    """Update the env_command_information with the predicted future trajectories.
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
                    ].future_trajectory = env_future_trajectory[agent_type][agent_id][
                        ITE_control_cmds[agent_type][agent_id].info.get("mode")
                    ]
                    logger.info(
                        f"agent_id: {agent_id} is updated to trajectory command with mode: {ITE_control_cmds[agent_type][agent_id].info.get('mode')}, trajectory: {ITE_control_cmds[agent_type][agent_id].future_trajectory}"
                    )
    return ITE_control_cmds