from addict import Dict
from loguru import logger

from terasim.params import AgentType
from terasim.overlay import traci, profile
from terasim_nde_nade.utils import is_intersect


def avoidable_maneuver_challenge_hook(veh_id):
    traci.vehicle.highlight(veh_id, (255, 0, 0, 120), duration=0.1)

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

    if veh1_foe_lane_id_set.intersection(
        veh2_next_lane_id_set
    ) or veh2_foe_lane_id_set.intersection(veh1_next_lane_id_set):
        return True
    return False

def get_maneuver_challenge(
    adversarial_agent_id,
    adversarial_agent_future,
    adversarial_agent_type,
    all_normal_agent_future,
    normal_agent_type,
    env_observation,
    agent_ctx_dict,
    record_in_ctx=False,
    highlight_flag=True,
    buffer=0,
):
    """Get the maneuver challenge for the adversarial vehicle.

    Args:
        adversarial_agent_id (str): the id of the adversarial agent
        adversarial_agent_future (list): the future trajectory of the adversarial agent
        adversarial_agent_type (str): "vehicle" or "vulnerable_road_user"
        all_normal_agent_future (dict): all_normal_veh _future[veh_id] = future trajectory of the normal agent
        normal_agent_type (str): "vehicle" or "vulnerable_road_user"

    Returns:
        num_affected_vehicles (int): the number of vehicles that will be affected by the adversarial vehicle
        maneuver_challenge_info (dict): maneuver_challenge_info[veh_id] = 1 if the adversarial vehicle will affect the normal vehicle
    """
    # see if the one adversarial future will intersect with other normal futures
    final_collision_flag = False
    if adversarial_agent_future is not None and all_normal_agent_future is not None:
        for agent_id in all_normal_agent_future:
            if agent_id == adversarial_agent_id:
                continue
            if all_normal_agent_future[agent_id] is None:
                print(
                    f"agent_id: {agent_id}, all_normal_agent_future[agent_id]: {all_normal_agent_future[agent_id]}"
                )
            link_intersection_flag = is_link_intersect(
                env_observation[adversarial_agent_type][adversarial_agent_id],
                env_observation[normal_agent_type][agent_id],
            )
            if not link_intersection_flag:
                continue  # if the next link of the two vehicles are not intersected, then the two vehicles will not collide

            # if the adversarial_veh_id is the header of the normal_veh_id, then the two vehicles will not collide
            # TODO: traci.vehicle.getLeader() can not detect the leading vulnerable road user
            if (
                normal_agent_type == AgentType.VEHICLE
                and adversarial_agent_type == AgentType.VEHICLE
            ):
                leader = traci.vehicle.getLeader(agent_id)
                if leader is not None and leader[0] == adversarial_agent_id:
                    continue

            # collision_flag = is_intersect(
            #     adversarial_agent_future,
            #     all_normal_agent_future[agent_id],
            #     veh_length,
            #     tem_len,
            #     circle_r + buffer,
            # ) # TODO: consider different vehicle length between the two colliding vehicles or vehicle and vulnerable road user
            collision_flag = is_intersect(
                adversarial_agent_future,
                all_normal_agent_future[agent_id],
                env_observation[adversarial_agent_type][adversarial_agent_id]["ego"][
                    "length"
                ],
                env_observation[normal_agent_type][agent_id]["ego"]["length"],
                env_observation[adversarial_agent_type][adversarial_agent_id]["ego"][
                    "width"
                ],
                env_observation[normal_agent_type][agent_id]["ego"]["width"],
                adversarial_agent_type.value,
                normal_agent_type.value,
                buffer,
            )
            final_collision_flag = final_collision_flag or collision_flag
            if collision_flag and record_in_ctx:
                if "conflict_vehicle_list" not in agent_ctx_dict:
                    agent_ctx_dict["conflict_vehicle_list"] = []
                agent_ctx_dict["conflict_vehicle_list"].append(agent_id)
                # logger.trace(
                #     f"veh_id: {adversarial_veh_id} will collide with veh_id: {veh_id}"
                # )
        return {
            "normal": 0,
            "adversarial": (1 if final_collision_flag else 0),
        }  # the first element is the number of vehicles that will be affected by the adversarial vehicle
    else:
        return {"normal": 0}
    
@profile
def get_maneuver_challenge_environment(env_future_trajectory, env_observation, env_command_information):
    """Get the maneuver challenge for each vehicle when it is in the adversarial mode while other vehicles are in the normal mode.
    Note: We only consider the challenge for the following cases:
    1. vehicle in the adversarial mode and the vehicle in the normal mode
    2. vru in the adversarial mode and the vehicle in the normal mode

    Args:
        trajectory_dicts (dict): trajectory_dicts[veh_id] = {"normal": normal_future_trajectory, "adversarial": adversarial_future_trajectory}

    Returns:
        maneuver_challenge_dict (dict): maneuver_challenge_dict[veh_id] = (num_affected_vehicles, affected_vehicles)
    """
    normal_future_trajectory_veh = Dict(
        {
            veh_id: env_future_trajectory[AgentType.VEHICLE][veh_id].get("normal", None)
            for veh_id in env_future_trajectory[AgentType.VEHICLE]
        }
    )
    adversarial_future_trajectory_veh = Dict(
        {
            veh_id: env_future_trajectory[AgentType.VEHICLE][veh_id].get(
                "adversarial", None
            )
            for veh_id in env_future_trajectory[AgentType.VEHICLE]
        }
    )
    adversarial_future_trajectory_vru = Dict(
        {
            vru_id: env_future_trajectory[AgentType.VULNERABLE_ROAD_USER][vru_id].get(
                "adversarial", None
            )
            for vru_id in env_future_trajectory[AgentType.VULNERABLE_ROAD_USER]
        }
    )

    # get the maneuver challenge for each vehicle, check if the adversarial future will collide with other vehicles' normal future
    maneuver_challenge_veh = Dict(
        {
            veh_id: get_maneuver_challenge(
                veh_id,
                adversarial_future_trajectory_veh[veh_id],
                AgentType.VEHICLE,
                normal_future_trajectory_veh,
                AgentType.VEHICLE,
                env_observation,
                env_command_information[AgentType.VEHICLE][veh_id],
                record_in_ctx=True,
            )
            for veh_id in env_future_trajectory[AgentType.VEHICLE]
        }
    )
    maneuver_challenge_dicts_vru = Dict(
        {
            vru_id: get_maneuver_challenge(
                vru_id,
                adversarial_future_trajectory_vru[vru_id],
                AgentType.VULNERABLE_ROAD_USER,
                normal_future_trajectory_veh,
                AgentType.VEHICLE,
                env_observation,
                env_command_information[AgentType.VULNERABLE_ROAD_USER][vru_id],
                record_in_ctx=True,
            )
            for vru_id in env_future_trajectory[AgentType.VULNERABLE_ROAD_USER]
        }
    )

    for veh_id in env_command_information[AgentType.VEHICLE]:
        env_command_information[AgentType.VEHICLE][veh_id]["maneuver_challenge"] = (
            maneuver_challenge_veh[veh_id]
            if veh_id in maneuver_challenge_veh
            else {"normal": 0}
        )

    for vru_id in env_command_information[AgentType.VULNERABLE_ROAD_USER]:
        env_command_information[AgentType.VULNERABLE_ROAD_USER][vru_id]["maneuver_challenge"] = (
            maneuver_challenge_dicts_vru[vru_id]
            if vru_id in maneuver_challenge_dicts_vru
            else {"normal": 0}
        )

    maneuver_challenge_veh_shrinked = Dict(
        {
            veh_id: maneuver_challenge_veh[veh_id]
            for veh_id in maneuver_challenge_veh
            if maneuver_challenge_veh[veh_id].get("adversarial")
        }
    )
    for veh_id in maneuver_challenge_veh_shrinked:
        avoidable_maneuver_challenge_hook(veh_id)
    conflict_vehicle_info = Dict(
        {
            AgentType.VEHICLE: {
                veh_id: env_command_information[AgentType.VEHICLE][veh_id].get(
                    "conflict_vehicle_list"
                )
                for veh_id in env_command_information[AgentType.VEHICLE]
                if env_command_information[AgentType.VEHICLE][veh_id].get("conflict_vehicle_list")
            },
            AgentType.VULNERABLE_ROAD_USER: {
                vru_id: env_command_information[AgentType.VULNERABLE_ROAD_USER][vru_id].get(
                    "conflict_vehicle_list"
                )
                for vru_id in env_command_information[AgentType.VULNERABLE_ROAD_USER]
                if env_command_information[AgentType.VULNERABLE_ROAD_USER][vru_id].get(
                    "conflict_vehicle_list"
                )
            },
        }
    )
    logger.trace(
        f"maneuver_challenge: {maneuver_challenge_veh_shrinked}, conflict_vehicle_info: {conflict_vehicle_info}"
    )
    env_maneuver_challenge = {
        AgentType.VEHICLE: maneuver_challenge_veh,
        AgentType.VULNERABLE_ROAD_USER: maneuver_challenge_dicts_vru,
    }
    return env_maneuver_challenge, env_command_information

