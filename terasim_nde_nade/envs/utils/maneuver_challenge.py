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
    
@profile
def get_maneuver_challenge_dicts(trajectory_dicts, obs_dicts, ctx_dicts):
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
            veh_id: get_maneuver_challenge(
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
            vru_id: get_maneuver_challenge(
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
        avoidable_maneuver_challenge_hook(veh_id)
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

