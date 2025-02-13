import random

import terasim.utils as utils
from addict import Dict
from loguru import logger
from terasim.agent.agent_controller import AgentController
from terasim.overlay import traci

from terasim_nde_nade.utils import (
    CommandType,
    NDECommand,
    TrajectoryPoint,
    interpolate_future_trajectory,
)


def get_all_routes():
    return traci.route.getIDList()


def get_all_route_edges():
    all_routes = get_all_routes()
    all_route_edges = {}
    for route in all_routes:
        all_route_edges[route] = traci.route.getEdges(route)
    return all_route_edges


class NDEVulnerableRoadUserController(AgentController):
    def __init__(self, simulator, params=None):
        self.is_busy = False
        self.cached_control_command = None  # this is a dict, containing the control command for the vehicle with the timestep information
        self.used_to_be_busy = False
        return super().__init__(
            simulator, control_command_schema=NDECommand, params=params
        )

    def _update_controller_status(self, vru_id, current_time=None):
        """Refresh the state of the controller. This function will be called at each timestep as far as vehicle is still in the simulator, even if the vehicle is not controlled."""
        # if the controller is busy, detect if the current simulation time - the time of the cached control command is greater than the duration of the control command, then the controller is not busy anymore
        if self.is_busy:
            current_time = (
                traci.simulation.getTime() if current_time is None else current_time
            )
            if (
                current_time - self.cached_control_command.timestep
                > self.cached_control_command.cached_command.duration
            ):
                self.is_busy = False
                self.cached_control_command = None
                self.used_to_be_busy = True

    def execute_control_command(self, vru_id, control_command, obs_dict):
        """Vehicle acts based on the input action.

        Args:
            action (dict): Lonitudinal and lateral actions. It should have the format: {'longitudinal': float, 'lateral': str}. The longitudinal action is the longitudinal acceleration, which should be a float. The lateral action should be the lane change direction. 'central' represents no lane change. 'left' represents left lane change, and 'right' represents right lane change.
        """
        if self.used_to_be_busy:
            traci.person.remove(vru_id)
            return
        if not self.is_busy:
            if control_command.command_type == CommandType.DEFAULT:
                # all_checks_on(veh_id)
                return
            elif control_command.command_type == CommandType.CUSTOM:
                self.cached_control_command = Dict(
                    {
                        "timestep": traci.simulation.getTime(),
                        "cached_command": control_command,
                    }
                )
                self.execute_control_command_onestep(
                    vru_id, self.cached_control_command, obs_dict, first_step=True
                )
                return
            else:
                # other commands will have duration, which will keep the controller busy
                self.is_busy = True
                # if the control command is a trajectory, then interpolate the trajectory
                control_command = interpolate_control_command(control_command)
                self.cached_control_command = Dict(
                    {
                        "timestep": traci.simulation.getTime(),
                        "cached_command": control_command,
                    }
                )
                self.execute_control_command_onestep(
                    vru_id, self.cached_control_command, obs_dict, first_step=True
                )
        else:
            self.execute_control_command_onestep(
                vru_id, self.cached_control_command, obs_dict, first_step=False
            )

    def execute_control_command_onestep(
        self, veh_id, cached_control_command, obs_dict, first_step=False
    ):
        if cached_control_command["cached_command"].command_type == CommandType.CUSTOM:
            if (
                cached_control_command["cached_command"].custom_control_command
                is not None
                and cached_control_command[
                    "cached_command"
                ].custom_execute_control_command
                is not None
            ):
                cached_control_command["cached_command"].custom_execute_control_command(
                    veh_id,
                    cached_control_command["cached_command"].custom_control_command,
                    obs_dict,
                )
                return
            else:
                logger.error(
                    "Custom control command or Custom control command execution is not defined"
                )
                return

        if (
            cached_control_command["cached_command"].command_type
            == CommandType.TRAJECTORY
        ):
            # pass
            self.execute_trajectory_command(
                veh_id, cached_control_command["cached_command"], obs_dict
            )
        elif (
            cached_control_command["cached_command"].command_type == CommandType.LEFT
            or cached_control_command["cached_command"].command_type
            == CommandType.RIGHT
        ):
            self.execute_lane_change_command(
                veh_id,
                cached_control_command["cached_command"],
                obs_dict,
                first_step=first_step,
            )
        elif (
            cached_control_command["cached_command"].command_type
            == CommandType.ACCELERATION
        ):
            self.execute_acceleration_command(
                veh_id, cached_control_command["cached_command"], obs_dict
            )
        else:
            logger.error("Invalid command type")
        return

    @staticmethod
    def execute_trajectory_command(veh_id, control_command, obs_dict):
        assert control_command.command_type == CommandType.TRAJECTORY
        # get the closest timestep trajectory point in control_command.trajectory to current timestep
        trajectory_array = control_command.future_trajectory
        current_timestep = traci.simulation.getTime()
        closest_timestep_trajectory = min(
            trajectory_array, key=lambda x: abs(x[-1] - current_timestep)
        )
        # set the position of the vehicle to the closest timestep trajectory point
        traci.person.moveToXY(
            personID=veh_id,
            edgeID="",
            x=closest_timestep_trajectory[0],
            y=closest_timestep_trajectory[1],
            angle=closest_timestep_trajectory[2],
            keepRoute=control_command.keep_route_mode,
        )
        logger.info(
            f"Setting position of {veh_id} to {closest_timestep_trajectory[0], closest_timestep_trajectory[1]}, current position is {traci.person.getPosition(veh_id)}"
        )


def interpolate_control_command(control_command):
    if control_command.command_type == CommandType.TRAJECTORY:
        # control_command.future_trajectory = interpolate_future_trajectory(
        #     control_command.future_trajectory, 0.1
        # )  # TODO: Angle cannot be interpolated
        return control_command
    else:
        return control_command
