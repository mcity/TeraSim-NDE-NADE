import addict

from terasim.overlay import traci
from terasim_nde_nade.utils import (
    CommandType,
    NDECommand,
    is_car_following,
)
from terasim_nde_nade.adversity.adversity.obs_processing import get_ff_acceleration, get_cf_acceleration


def derive_leader_negligence_command(obs_dict, highlight_flag=False, highlight_color=[255, 0, 0, 255]) -> addict.Dict:
    leader_info = traci.vehicle.getLeader(obs_dict["ego"]["veh_id"], 40)
    current_acceleration = obs_dict["ego"]["acceleration"]
    ff_acceleration = get_ff_acceleration(obs_dict)

    # get negligence command candidates
    negligence_command_dict = addict.Dict()
    cf_acceleration = current_acceleration

    if (
        leader_info is not None
    ):  # there is a leading vehicle, add lead neglgience type
        cf_acceleration = get_cf_acceleration(obs_dict, leader_info)
        # if the vehicle and the leading vehicle are both stopped, disable negligence
        if (
            obs_dict["ego"]["velocity"] < 0.5
            and traci.vehicle.getSpeed(leader_info[0]) < 0.5
        ):
            return negligence_command_dict

        # if the vehicle is car following
        is_car_following_flag = is_car_following(obs_dict["ego"]["veh_id"], leader_info[0])
        if is_car_following_flag:
            leader_velocity = traci.vehicle.getSpeed(leader_info[0])
            # ego vehicle is stopping or the velocity difference between the ego vehicle and the leader is small
            if (
                obs_dict["ego"]["velocity"] < 0.5
                or abs(obs_dict["ego"]["velocity"] - leader_velocity) < 2
            ):
                return negligence_command_dict

        # if the free flow acceleration is significantly larger than the car following accelerations
        if ff_acceleration - cf_acceleration > 1.5:
            negligence_command = NDECommand(
                command_type=CommandType.ACCELERATION,
                duration=2.0,
                acceleration=ff_acceleration,
            )
            negligence_command.info.update(
                {
                    "is_car_following_flag": is_car_following_flag,
                    "leader_info": leader_info,
                    "ff_acceleration": ff_acceleration,
                    "cf_acceleration": cf_acceleration,
                    "current_acceleration": current_acceleration,
                    "mode": "negligence",
                    "negligence_mode": "Lead",
                }
            )
            negligence_command_dict.update(addict.Dict({"Lead": negligence_command}))
            if highlight_flag:
                traci.vehicle.setColor(
                    obs_dict["ego"]["veh_id"], highlight_color
                )  # highlight the vehicle with red
        return negligence_command_dict
    else:
        return negligence_command_dict
