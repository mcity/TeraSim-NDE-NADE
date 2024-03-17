from terasim.agent.agent_controller import AgentController
import terasim.utils as utils
import random
from .nde_vehicle_utils import (
    get_next_lane_edge,
    get_neighbour_lane,
    Command,
    NDECommand,
    TrajectoryPoint,
    interpolate_future_trajectory,
)
from terasim.overlay import traci
from addict import Dict


def get_all_routes():
    return traci.route.getIDList()


def get_all_route_edges():
    all_routes = get_all_routes()
    all_route_edges = {}
    for route in all_routes:
        all_route_edges[route] = traci.route.getEdges(route)
    return all_route_edges


class NDEController(AgentController):

    def __init__(self, simulator, params):
        self.is_busy = False
        self.cached_control_command = None  # this is a dict, containing the control command for the vehicle with the timestep information
        return super().__init__(
            simulator, control_command_schema=NDECommand, params=params
        )

    def _update_controller_status(self, veh_id, current_time=None):
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
                self.all_checks_on(veh_id)

    # TODO: combine this with the execute_control_command
    def execute_urban_lanechange(self, veh_id, control_command, obs_dict):
        self.allow_lane_change = False  # disable urban lane change after 1 lane change
        vehicle_lane_id = obs_dict["ego"]["lane_id"]
        neighbour_lane = get_neighbour_lane(
            self.simulator.sumo_net,
            vehicle_lane_id,
            direction=control_command["lateral"],
        )
        if neighbour_lane is not None:
            current_edge_id = obs_dict["local"]["Ego"]["edge_id"]
            new_target_lane, new_target_edge = get_next_lane_edge(
                self.simulator.sumo_net, neighbour_lane
            )
            all_route_edges = get_all_route_edges()
            perfect_new_route_index = None
            for route in all_route_edges.keys():
                if (
                    new_target_edge in all_route_edges[route]
                    and current_edge_id in all_route_edges[route]
                ):
                    perfect_new_route_index = route
                    break
            if (
                perfect_new_route_index is not None
            ):  # if there is a route that can reach the target edge and the current edge, then use this route
                traci.vehicle.setRouteID(veh_id, route)
            else:
                final_edge_id_in_new_route = None
                for (
                    route
                ) in (
                    all_route_edges.keys()
                ):  # if there is no route that can reach the target edge and the current edge, then find a route that can reach the target edge and let the vehicle go to the final edge of this route
                    if new_target_edge in all_route_edges[route]:
                        final_edge_id_in_new_route = all_route_edges[route][-1]
                        break
                # if no route can reach the target edge, then let the vehicle go to the target edge
                final_edge_id_in_new_route = (
                    final_edge_id_in_new_route
                    if final_edge_id_in_new_route is not None
                    else new_target_edge
                )
                traci.vehicle.changeTarget(veh_id, new_target_edge)

    def is_urban_lanechange(self, veh_edge_id):
        return (
            ("EG_1_1_1" not in veh_edge_id)
            and ("EG_1_3_1" not in veh_edge_id)
            and ("NODE_7_0" not in veh_edge_id)
        )

    def execute_control_command(self, veh_id, control_command, obs_dict):
        """Vehicle acts based on the input action.

        Args:
            action (dict): Lonitudinal and lateral actions. It should have the format: {'longitudinal': float, 'lateral': str}. The longitudinal action is the longitudinal acceleration, which should be a float. The lateral action should be the lane change direction. 'central' represents no lane change. 'left' represents left lane change, and 'right' represents right lane change.
        """
        if not self.is_busy:
            if control_command.command_type == Command.DEFAULT:
                # all_checks_on(veh_id)
                return
            else:
                self.all_checks_off(veh_id)
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
        if (
            self.cached_control_command["cached_command"].command_type
            == Command.TRAJECTORY
        ):
            # pass
            self.execute_trajectory_command(
                veh_id, self.cached_control_command["cached_command"], obs_dict
            )
        elif (
            self.cached_control_command["cached_command"].command_type == Command.LEFT
            or self.cached_control_command["cached_command"].command_type
            == Command.RIGHT
        ):
            self.execute_lane_change_command(
                veh_id, self.cached_control_command["cached_command"], obs_dict
            )
        elif (
            self.cached_control_command["cached_command"].command_type
            == Command.ACCELERATION
        ):
            self.execute_acceleration_command(
                veh_id, self.cached_control_command["cached_command"], obs_dict
            )
        else:
            pass
        return

    def execute_trajectory_command(self, veh_id, control_command, obs_dict):
        assert control_command.command_type == Command.TRAJECTORY
        # get the closest timestep trajectory point in control_command.trajectory to current timestep
        trajectory_array = control_command.future_trajectory
        current_timestep = traci.simulation.getTime()
        closest_timestep_trajectory = min(
            trajectory_array, key=lambda x: abs(x[-1] - current_timestep)
        )
        # set the position of the vehicle to the closest timestep trajectory point
        traci.vehicle.moveToXY(
            vehID=veh_id,
            edgeID="",
            laneIndex=-1,
            x=closest_timestep_trajectory[0],
            y=closest_timestep_trajectory[1],
            angle=closest_timestep_trajectory[2],
            keepRoute=1,
        )
        traci.vehicle.setPreviousSpeed(veh_id, closest_timestep_trajectory[3])

    @staticmethod
    def execute_lane_change_command(veh_id, control_command, obs_dict):
        assert (
            control_command.command_type == Command.LEFT
            or control_command.command_type == Command.RIGHT
        )
        indexOffset = 1 if control_command.command_type == Command.LEFT else -1
        traci.vehicle.changeLaneRelative(veh_id, indexOffset, utils.get_step_size())

    @staticmethod
    def execute_acceleration_command(veh_id, control_command, obs_dict):
        assert control_command.command_type == Command.ACCELERATION
        acceleration = control_command.acceleration
        final_speed = obs_dict["ego"]["velocity"] + acceleration * utils.get_step_size()
        final_speed = 0 if final_speed < 0 else final_speed
        traci.vehicle.setSpeed(veh_id, final_speed)

    @staticmethod
    def all_checks_on(veh_id):
        traci.vehicle.setSpeedMode(veh_id, 31)
        traci.vehicle.setLaneChangeMode(veh_id, 1621)

    @staticmethod
    def all_checks_off(veh_id):
        traci.vehicle.setSpeedMode(veh_id, 32)
        traci.vehicle.setLaneChangeMode(veh_id, 0)


def interpolate_control_command(control_command):
    if control_command.command_type == Command.TRAJECTORY:
        control_command.future_trajectory = interpolate_future_trajectory(
            control_command.future_trajectory, 0.1
        )
        return control_command
    else:
        return control_command
