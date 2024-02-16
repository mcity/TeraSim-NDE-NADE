from terasim.vehicle.controllers.high_efficiency_controller import HighEfficiencyController
import terasim.utils as utils
import random
from .vehicle_utils import get_location, get_next_lane_edge, get_neighbour_lane
from terasim.overlay import traci

def get_all_routes():
    return traci.route.getIDList()

def get_all_route_edges():
    all_routes = get_all_routes()
    all_route_edges = {}
    for route in all_routes:
        all_route_edges[route] = traci.route.getEdges(route)
    return all_route_edges

class HighEfficiencyControllerITE(HighEfficiencyController):
    params = {
            "v_high": 20,
            "v_low": 0,
            "acc_duration": 0.1,  # the acceleration duration will be 0.1 second
            "lc_duration": 1,  # the lane change duration will be 1 second
            "neg_duration": 2, # the negligence duration will be 2 second
            "avoid_collision_duration": 3, # the duration to avoid collision will be 2 second
        }
    
    def __init__(self, simulator, params=None):
        super().__init__(simulator, params)
        self.neg_step_num = int(self.params["neg_duration"]/self.step_size)
        self.avoid_collision_step_num = int(self.params["avoid_collision_duration"]/self.step_size)
        self.allow_lane_change = True
        self.avoid_collision_flag = False
    
    def _is_command_legal(self, veh_id, control_command):
        """Check if the control command is legal.
        """
        if control_command["lateral"] != "central" and not self.allow_lane_change:
            return False
        if control_command["type"] != "lon_lat" or "longitudinal" not in control_command or "lateral" not in control_command:
            raise ValueError("The control command should have the format: {'longitudinal': float, 'lateral': str}. The longitudinal action is the longitudinal acceleration, which should be a float. The lateral action should be the lane change direction. 'central' represents no lane change. 'left' represents left lane change, and 'right' represents right lane change.")
        
        if self.is_busy:
            assert self.controlled_duration >= 0
            if self.avoid_collision_flag:
                # print(f"avoid collision: {veh_id}, {self.controlled_duration}, {traci.vehicle.getAcceleration(veh_id)}, {traci.vehicle.getSpeed(veh_id)}")
                self.change_vehicle_speed(veh_id, -7.06, 0.1)
            if self.controlled_duration == 0:
                self.is_busy = False
                self.avoid_collision_flag = False
            else:
                self.controlled_duration -= 1
                self.controlled_duration = max(self.controlled_duration, 0)
                
            return False
        else:
            if control_command["lateral"] == "left" and not self.simulator.get_vehicle_lane_adjacent(veh_id, 1):
                return False
            if control_command["lateral"] == "right" and not self.simulator.get_vehicle_lane_adjacent(veh_id, -1):
                return False
            return True
    
    def execute_urban_lanechange(self, veh_id, control_command, obs_dict):
        self.allow_lane_change = False # disable urban lane change after 1 lane change
        vehicle_lane_id = obs_dict["ego"].data["lane_id"]
        neighbour_lane = get_neighbour_lane(self.simulator.sumo_net, vehicle_lane_id, direction = control_command["lateral"])
        if neighbour_lane is not None:
            current_edge_id = obs_dict["local"].data["Ego"]["edge_id"]
            new_target_lane, new_target_edge = get_next_lane_edge(self.simulator.sumo_net, neighbour_lane)
            all_route_edges = get_all_route_edges()
            perfect_new_route_index = None
            for route in all_route_edges.keys():
                if new_target_edge in all_route_edges[route] and current_edge_id in all_route_edges[route]:
                    perfect_new_route_index = route
                    break
            if perfect_new_route_index is not None: # if there is a route that can reach the target edge and the current edge, then use this route
                traci.vehicle.setRouteID(veh_id, route)
            else:
                final_edge_id_in_new_route = None
                for route in all_route_edges.keys(): # if there is no route that can reach the target edge and the current edge, then find a route that can reach the target edge and let the vehicle go to the final edge of this route
                    if new_target_edge in all_route_edges[route]:
                        final_edge_id_in_new_route = all_route_edges[route][-1]
                        break
                # if no route can reach the target edge, then let the vehicle go to the target edge
                final_edge_id_in_new_route = final_edge_id_in_new_route if final_edge_id_in_new_route is not None else new_target_edge
                traci.vehicle.changeTarget(veh_id, new_target_edge)

    def is_urban_lanechange(self, veh_edge_id):
        return ("EG_1_1_1" not in veh_edge_id) and ("EG_1_3_1" not in veh_edge_id) and ("NODE_7_0" not in veh_edge_id)

    def execute_control_command(self, veh_id, control_command, obs_dict):
        """Vehicle acts based on the input action.

        Args:
            action (dict): Lonitudinal and lateral actions. It should have the format: {'longitudinal': float, 'lateral': str}. The longitudinal action is the longitudinal acceleration, which should be a float. The lateral action should be the lane change direction. 'central' represents no lane change. 'left' represents left lane change, and 'right' represents right lane change.
        """ 
        # Set Safe Mode from SUMO
        # if control_command["longitudinal"] == "SUMO":
        #     # utils.set_vehicle_speedmode(veh_id)
        #     return

        negligence_flag = "mode" in control_command and control_command["mode"] == "negligence"
        avoid_collision_flag = "mode" in control_command and control_command["mode"] == "avoid_collision"

        # # disable MOBIL lane change
        # control_command["lateral"] = "central"
        # utils.set_vehicle_speedmode(veh_id, 0)
        if not self.is_busy:    
            if negligence_flag or avoid_collision_flag:
                utils.set_vehicle_speedmode(veh_id, 0)
                if not self.is_busy:
                    self.is_busy = True
                    if negligence_flag:
                        self.controlled_duration = self.neg_step_num  # begin counting the negligence timesteps
                    elif avoid_collision_flag:
                        self.avoid_collision_flag = True
                        self.controlled_duration = self.avoid_collision_step_num
            else:
                utils.set_vehicle_speedmode(veh_id)

        # if avoid_collision_flag:
        #     print(f"{veh_id}, acceleration: {control_command['longitudinal']}, {traci.vehicle.getAcceleration(veh_id)}, {self.controlled_duration}")

        # Longitudinal control
        if control_command["longitudinal"] == "SUMO":
            utils.set_vehicle_speedmode(veh_id)
            controlled_acc = None
        else:
            controlled_acc = control_command["longitudinal"]
            current_velocity = obs_dict["ego"].data["velocity"]
            if current_velocity + controlled_acc > self.params["v_high"]:
                controlled_acc = self.params["v_high"] - current_velocity
            # elif current_velocity + controlled_acc < self.params["v_low"]:
            #     controlled_acc = self.params["v_low"] - current_velocity
        if negligence_flag:
            duration = self.params["neg_duration"]
        elif "duration" in control_command:
            duration = control_command["duration"]
        else:
            duration = self.params["acc_duration"]
        if control_command["lateral"] == "SUMO":
            utils.set_vehicle_lanechangemode(veh_id)
            if controlled_acc is not None:
                self.change_vehicle_speed(veh_id, controlled_acc, duration)
        else:
            if control_command["lateral"] == "central":
                utils.set_vehicle_lanechangemode(veh_id, 512) # disable lane change in central mode
                if controlled_acc is not None:
                    self.change_vehicle_speed(veh_id, controlled_acc, duration)
            else:
                lane_change_mode = 0 if utils.get_distance(veh_id) > 10 else 512
                utils.set_vehicle_lanechangemode(veh_id, lane_change_mode)
                utils.set_vehicle_speedmode(veh_id)
                self.simulator.change_vehicle_lane(veh_id, control_command["lateral"], self.params["lc_duration"])
                if controlled_acc is not None:
                    self.change_vehicle_speed(veh_id, controlled_acc, self.params["lc_duration"])
                self.controlled_duration = self.lc_step_num + random.randint(5, 10) # begin counting the lane change maneuver timesteps
                self.is_busy = True
                vehicle_edge_id = obs_dict["local"].data["Ego"]["edge_id"]
                if self.is_urban_lanechange(vehicle_edge_id):
                    self.execute_urban_lanechange(veh_id, control_command, obs_dict)

    def calculate_decelerate_duration(self, veh_id, deceleration):
        """Calculate the duration to decelerate to a certain speed.

        Args:
            veh_id (str): Vehicle ID
            deceleration (float): Specified deceleration of vehicle.

        Returns:
            float: The duration to decelerate to a certain speed.
        """        
        current_speed = traci.vehicle.getSpeed(veh_id)
        if deceleration >= 0:
            return 1
        else:
            return max(1, int(-current_speed/deceleration * 10)+1)

    def change_vehicle_speed(self, vehID, acceleration, duration=1.0):
        """Fix the acceleration of a vehicle to be a specified value in the specified duration.

        Args:
            vehID (str): Vehicle ID
            acceleration (float): Specified acceleration of vehicle.
            duration (float, optional): Specified time interval to fix the acceleration in s. Defaults to 1.0.
        """
        init_speed = traci.vehicle.getSpeed(vehID)
        final_speed = init_speed + acceleration * duration
        if final_speed < 0:
            final_speed = 0
        if duration == utils.get_step_size():
            traci.vehicle.setSpeed(vehID, final_speed)
        else:
            traci.vehicle.slowDown(vehID, final_speed, duration)