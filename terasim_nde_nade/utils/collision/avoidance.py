from loguru import logger

from terasim.overlay import traci
from ..base import CommandType, NDECommand, get_adversity_pair_dict
from ..trajectory import predict_future_trajectory_vehicle

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


def add_avoid_accept_collision_command(
    sumo_net, env_observation, trajectory_dicts, command_info_env
):
    potential_adversity_pair_dict = get_adversity_pair_dict(
        command_info_env, potential=True
    )
    # add avoidance command for the neglected vehicles
    for (
        neglecting_vehicle_id,
        neglected_vehicle_list,
    ) in potential_adversity_pair_dict.vehicle.items():
        neglecting_vehicle_future = trajectory_dicts.vehicle[
            neglecting_vehicle_id
        ]["adversarial"]
        for neglected_vehicle_id in neglected_vehicle_list:
            neglected_vehicle_future = trajectory_dicts.vehicle[
                neglected_vehicle_id
            ]["normal"]
            avoidance_command = get_vehicle_avoidance_command(
                neglecting_vehicle_future,
                neglected_vehicle_future,
                neglecting_vehicle_id,
                neglected_vehicle_id,
                traci.vehicle.getEmergencyDecel(neglected_vehicle_id)
            )
            command_info_env.vehicle[neglected_vehicle_id][
                "ndd_command_distribution"
            ]["avoid_collision"] = avoidance_command
            (
                trajectory_dicts.vehicle[neglected_vehicle_id][
                    "avoid_collision"
                ],
                info,
            ) = predict_future_trajectory_vehicle(
                neglected_vehicle_id,
                env_observation.vehicle[neglected_vehicle_id],
                avoidance_command,
                sumo_net,
                time_horizon_step=5,
                time_resolution=0.5,
                interpolate_resolution=0.5,
                current_time=None,
                veh_info=None,
            )

            accept_command = get_accept_collision_command()
            command_info_env.vehicle[neglected_vehicle_id][
                "ndd_command_distribution"
            ]["accept_collision"] = accept_command
            (
                trajectory_dicts.vehicle[neglected_vehicle_id][
                    "accept_collision"
                ],
                info,
            ) = predict_future_trajectory_vehicle(
                neglected_vehicle_id,
                env_observation.vehicle[neglected_vehicle_id],
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
    ) in potential_adversarial_pair_dict.vru.items():
        neglecting_vru_future = trajectory_dicts.vru[
            neglecting_vru_id
        ]["adversarial"]
        for neglected_vehicle_id in neglected_vehicle_list:
            if (
                command_info_env.vehicle[neglected_vehicle_id][
                    "ndd_command_distribution"
                ].get("avoid_collision", None)
                is not None
            ):
                continue
            neglected_vehicle_future = trajectory_dicts.vehicle[
                neglected_vehicle_id
            ]["normal"]
            avoidance_command = get_vehicle_avoidance_command(
                neglecting_vru_future,
                neglected_vehicle_future,
                neglecting_vru_id,
                neglected_vehicle_id,
                traci.vehicle.getEmergencyDecel(neglected_vehicle_id)
            )
            command_info_env.vehicle[neglected_vehicle_id][
                "ndd_command_distribution"
            ]["avoid_collision"] = avoidance_command
            (
                trajectory_dicts.vehicle[neglected_vehicle_id][
                    "avoid_collision"
                ],
                info,
            ) = predict_future_trajectory_vehicle(
                neglected_vehicle_id,
                env_observation.vehicle[neglected_vehicle_id],
                avoidance_command,
                sumo_net,
                time_horizon_step=5,
                time_resolution=0.5,
                interpolate_resolution=0.5,
                current_time=None,
                veh_info=None,
            )

            accept_command = get_accept_collision_command()
            command_info_env.vehicle[neglected_vehicle_id][
                "ndd_command_distribution"
            ]["accept_collision"] = accept_command
            (
                trajectory_dicts.vehicle[neglected_vehicle_id][
                    "accept_collision"
                ],
                info,
            ) = predict_future_trajectory_vehicle(
                neglected_vehicle_id,
                env_observation.vehicle[neglected_vehicle_id],
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
    return trajectory_dicts, command_info_env
