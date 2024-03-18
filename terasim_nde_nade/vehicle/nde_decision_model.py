from terasim.vehicle.decision_models.idm_model import IDMModel
import terasim.utils as utils
import numpy as np
import logging
from terasim.overlay import traci
import os
import json
import terasim_nde_nade.vehicle.nde_vehicle_utils as nde_utils
from terasim_nde_nade.vehicle.nde_vehicle_utils import (
    get_collision_type_and_prob,
    Command,
    NDECommand,
    TrajectoryPoint,
    is_car_following,
    get_location,
)
import attr
import random
import traci.constants as tc
from typing import List, Tuple
from addict import Dict


class NDEDecisionModel(IDMModel):

    def __init__(
        self,
        MOBIL_lc_flag=True,
        stochastic_acc_flag=False,
        IDM_parameters=None,
        MOBIL_parameters=None,
    ):
        current_path = os.path.dirname(os.path.abspath(__file__))
        lane_config_path = os.path.join(current_path, "lane_config.json")
        self.lane_config = json.load(open(lane_config_path, "r"))
        super().__init__(
            MOBIL_lc_flag, stochastic_acc_flag, IDM_parameters, MOBIL_parameters
        )

    def change_vehicle_type_according_to_location(self, veh_id, vehicle_location):
        if "highway" in vehicle_location:  # highway/freeway scenario
            traci.vehicle.setType(veh_id, "NDE_HIGHWAY")
        elif (
            "intersection" in vehicle_location or "roundabout" in vehicle_location
        ):  # urban scenario
            traci.vehicle.setType(veh_id, "NDE_URBAN")
        else:
            raise ValueError(f"location {vehicle_location} not supported")

    # TODO: check if we should use rerouteeffort or changetarget
    @staticmethod
    def reroute_vehicle_if_necessary(veh_id: str, current_lane_id: str) -> bool:
        bestlanes: List[Tuple[str, float, float, bool]] = traci.vehicle.getBestLanes(
            veh_id
        )
        # get the bestlane with current lane id
        for lane in bestlanes:
            if lane[0] == current_lane_id:
                if not lane[4]:  # the current lane does not allow continueing the route
                    traci.vehicle.rerouteEffort(veh_id)
                    return True  # reroute the vehicle
        return False  # do not reroute the vehicle

    def derive_control_command_from_observation(self, obs_dict):
        # change the IDM and MOBIL parameters based on the location
        vehicle_location = get_location(obs_dict["ego"]["edge_id"])
        self.change_vehicle_type_according_to_location(
            obs_dict["ego"]["veh_id"], vehicle_location
        )
        self.reroute_vehicle_if_necessary(
            obs_dict["ego"]["veh_id"], obs_dict["ego"]["lane_id"]
        )

        # set yellow color for the vehicle
        traci.vehicle.setColor(obs_dict["ego"]["veh_id"], (255, 255, 0, 255))

        leader_info = traci.vehicle.getLeader(obs_dict["ego"]["veh_id"], 40)
        current_acceleration = obs_dict["ego"]["acceleration"]
        ff_acceleration = get_ff_acceleration(obs_dict)

        # get negligence command candidates
        negligence_command_dict = Dict()
        cf_acceleration = current_acceleration
        if (
            leader_info is not None
        ):  # there is a leading vehicle, add lead neglgience type
            cf_acceleration = get_cf_acceleration(obs_dict, leader_info)
            negligence_command_dict.update(
                Dict(
                    leader_negligence(
                        obs_dict,
                        leader_info,
                        ff_acceleration,
                        cf_acceleration,
                        current_acceleration,
                        highlight_flag=True,
                    )
                )
            )
        if (
            leader_info is None or cf_acceleration - current_acceleration > 1.5
        ):  # the vehicle is constained by the traffic rules
            negligence_command_dict.update(
                Dict(
                    traffic_rule_negligence(
                        obs_dict,
                        ff_acceleration,
                        current_acceleration,
                        highlight_flag=True,
                    )
                )
            )
        negligence_command_dict.update(
            Dict(lane_change_negligence(obs_dict, highlight_flag=True))
        )

        # If there are no negligence commands, use the default command with probability 1
        command_dict = {"normal": NDECommand(command_type=Command.DEFAULT, prob=1)}

        # If there are negligence commands, update the command_dict with the negligence command and the normal command with the remaining probability
        negligence_command = None
        if negligence_command_dict:
            # traci.vehicle.setColor(obs_dict["ego"]["veh_id"], (255, 0, 0, 255)) # highlight the vehicle with red
            negligence_command = list(negligence_command_dict.values())[0]
            negligence_command.prob, predicted_collision_type = (
                get_collision_type_and_prob(
                    obs_dict, negligence_command, vehicle_location
                )
            )
            negligence_command.info.update(
                {
                    "predicted_collision_type": predicted_collision_type,
                    "vehicle_location": vehicle_location,
                }
            )
            command_dict = {
                "negligence": negligence_command,
                "normal": NDECommand(
                    command_type=Command.DEFAULT,
                    prob=1 - negligence_command.prob,
                    info={"vehicle_location": vehicle_location},
                ),
            }

        # sample final command based on the probability in command_dict
        command = random.choices(
            list(command_dict.values()),
            weights=[command_dict[key].prob for key in command_dict],
            k=1,
        )[0]
        # command = NDECommand(command_type=Command.DEFAULT, prob=1)
        return command, {"ndd_command_distribution": command_dict}


def will_stop_at_stopline(veh_id):
    # next_tls = traci.vehicle.getNextTLS(veh_id)
    next_links = traci.vehicle.getNextLinks(veh_id)

    if next_links and not next_links[0][1]:
        # if next link is not empty, and the vehicle does not have road priority, then the vehicle is stopping at the stopline
        # check if the vehicle is stopping at intersection stopline using next_links[0][5] which denotes the state of the link
        if next_links[0][5] in "GgRrYyw":
            # traci set vehicle brown color
            # traci.vehicle.setColor(veh_id, (139, 69, 19, 255))
            return True, "intersection"
        elif next_links[0][5] in "smM":
            # traci set vehicle purple color
            # traci.vehicle.setColor(veh_id, (128, 0, 128, 255))
            return True, "roundabout"
        else:
            return False, None
    else:
        return False, None


def traffic_rule_negligence(
    obs_dict, ff_acceleration, current_acceleration, highlight_flag=False
):

    negligence_command_dict = Dict()

    stopping, stop_location = will_stop_at_stopline(obs_dict["ego"]["veh_id"])
    if not stopping:
        return negligence_command_dict

    if ff_acceleration - current_acceleration > 1.0 or (
        current_acceleration < 0 and ff_acceleration > 0.2
    ):
        negligence_command_dict["TrafficRule"] = NDECommand(
            command_type=Command.ACCELERATION,
            duration=2.0,
            acceleration=ff_acceleration,
        )
        negligence_command_dict["TrafficRule"].info.update(
            {
                "mode": "negligence",
                "negligence_mode": "TrafficRule",
                "stopping": stopping,
                "stop_location": stop_location,
            }
        )

        if highlight_flag:
            traci.vehicle.setColor(
                obs_dict["ego"]["veh_id"], (0, 0, 255, 255)
            )  # highlight the vehicle with blue
    return negligence_command_dict


def leader_negligence(
    obs_dict,
    leader_info,
    ff_acceleration,
    cf_acceleration,
    current_acceleration,
    highlight_flag=False,
):
    negligence_command_dict = Dict()

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
            command_type=Command.ACCELERATION,
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
        negligence_command_dict.update(Dict({"Lead": negligence_command}))
        if highlight_flag:
            traci.vehicle.setColor(
                obs_dict["ego"]["veh_id"], (255, 0, 0, 255)
            )  # highlight the vehicle with red
    return negligence_command_dict


def wantsChangeLane(direction, state=None):
    """wantsAndCouldChangeLane(string, int) -> bool
    Return whether the vehicle wants to and could change lanes in the specified direction
    """
    if direction == -1:
        return state & tc.LCA_RIGHT != 0
    if direction == 1:
        return state & tc.LCA_LEFT != 0
    return False


def lane_change_negligence(obs_dict, highlight_flag=False):
    negligence_command_dict = {}
    left_lc_state = traci.vehicle.getLaneChangeStatePretty(
        obs_dict["ego"]["veh_id"], 1
    )[1]
    right_lc_state = traci.vehicle.getLaneChangeStatePretty(
        obs_dict["ego"]["veh_id"], -1
    )[1]
    left_lc_state_original = traci.vehicle.getLaneChangeState(
        obs_dict["ego"]["veh_id"], 1
    )
    right_lc_state_original = traci.vehicle.getLaneChangeState(
        obs_dict["ego"]["veh_id"], -1
    )

    # left_lc_will = wantsChangeLane(1, left_lc_state_original[1])
    # right_lc_will = wantsChangeLane(-1, right_lc_state_original[1])

    if (
        "blocked by left follower" in left_lc_state
        and "blocked by left leader" not in left_lc_state
    ):  # blocked only by left follower
        left_follower = traci.vehicle.getLeftFollowers(
            obs_dict["ego"]["veh_id"]
        )  # get the left follower

        if len(left_follower):  # the left follower is close to the ego vehicle
            follower_mingap = traci.vehicle.getMinGap(left_follower[0][0])
            if left_follower[0][1] + follower_mingap > -2:
                negligence_command_dict["LeftFoll"] = NDECommand(
                    command_type=Command.LEFT, duration=1.0
                )
                negligence_command_dict["LeftFoll"].info.update(
                    {"mode": "negligence", "negligence_mode": "LeftFoll"}
                )
                if highlight_flag:
                    traci.vehicle.setColor(
                        obs_dict["ego"]["veh_id"], (0, 255, 0, 255)
                    )  # highlight the vehicle with green
    if (
        "blocked by right follower" in right_lc_state
        and "blocked by right leader" not in right_lc_state
    ):  # blocked only by right follower
        right_follower = traci.vehicle.getRightFollowers(
            obs_dict["ego"]["veh_id"]
        )  # get the right follower
        if len(right_follower):
            follower_mingap = traci.vehicle.getMinGap(right_follower[0][0])
            # the right follower is close to the ego vehicle
            if right_follower[0][1] + follower_mingap > -2:
                negligence_command_dict["RightFoll"] = NDECommand(
                    command_type=Command.RIGHT, duration=1.0
                )
                negligence_command_dict["RightFoll"].info.update(
                    {"mode": "negligence", "detailed_mode": "RightFoll"}
                )
                if highlight_flag:
                    traci.vehicle.setColor(
                        obs_dict["ego"]["veh_id"], (0, 255, 0, 255)
                    )  # highlight the vehicle with green
    return negligence_command_dict


def get_ff_acceleration(obs_dict):
    ff_speed = min(
        traci.vehicle.getFollowSpeed(
            obs_dict["ego"]["veh_id"],
            obs_dict["ego"]["velocity"],
            3000,
            obs_dict["ego"]["velocity"],
            7.06,
        ),
        traci.vehicle.getAllowedSpeed(obs_dict["ego"]["veh_id"]),
    )
    return (ff_speed - obs_dict["ego"]["velocity"]) / traci.simulation.getDeltaT()


def get_cf_acceleration(obs_dict, leader_info):
    leader_id, leader_distance = leader_info
    cf_speed_with_leading_vehicle = traci.vehicle.getFollowSpeed(
        obs_dict["ego"]["veh_id"],
        obs_dict["ego"]["velocity"],
        leader_distance,
        traci.vehicle.getSpeed(leader_id),
        7.06,
    )
    cf_acceleration = (
        cf_speed_with_leading_vehicle - obs_dict["ego"]["velocity"]
    ) / traci.simulation.getDeltaT()
    return cf_acceleration
