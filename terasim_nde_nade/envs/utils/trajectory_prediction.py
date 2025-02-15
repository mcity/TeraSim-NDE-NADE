from addict import Dict
from loguru import logger

from terasim.overlay import profile, traci
from terasim.params import AgentType
from terasim_nde_nade.utils import (
    CommandType,
    get_vehicle_info,
    predict_future_trajectory,
)
from terasim_nde_nade.utils.agents.vru import (
    get_vulnerbale_road_user_info,
    predict_future_trajectory_vulnerable_road_user,
)


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

@profile
def predict_future_trajectory_dicts(sumo_net, obs_dicts, ctx_dicts):
    # predict future trajectories for each vehicle
    current_time = traci.simulation.getTime()
    trajectory_dicts = {
        AgentType.VEHICLE: {},
        AgentType.VULNERABLE_ROAD_USER: {},
    }
    # for vehicles
    ndd_control_command_dicts = get_ndd_distribution_from_ctx(
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
    ndd_control_command_dicts = get_ndd_distribution_from_ctx(
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
