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

class HighEfficiencyControllerV2(HighEfficiencyController):
    params = {
            "v_high": 20,
            "v_low": 0,
            "acc_duration": 0.1,  # the acceleration duration will be 0.1 second
            "lc_duration": 1,  # the lane change duration will be 1 second
            "neg_duration": 2, # the negligence duration will be 2 second
        }
    
    def __init__(self, simulator, params=None):
        super().__init__(simulator, params)
        self.neg_step_num = int(self.params["neg_duration"]/self.step_size)
        self.allow_lane_change = True

    def _is_command_legal(self, veh_id, control_command):
        if control_command["lateral"] != "central" and not self.allow_lane_change:
            return False
        is_legal = super()._is_command_legal(veh_id, control_command)
        return is_legal
    
    def execute_urban_lanechange(self, veh_id, control_command, obs_dict):
        self.allow_lane_change = False # disable urban lane change after 1 lane change
        vehicle_lane_id = obs_dict["local"].data["Ego"]["road_id"] + "_" + str(obs_dict["local"].data["Ego"]["lane_index"])
        neighbour_lane = get_neighbour_lane(self.simulator.sumo_net, vehicle_lane_id, direction = control_command["lateral"])
        if neighbour_lane is not None:
            current_edge_id = obs_dict["local"].data["Ego"]["road_id"]
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
        

    def execute_control_command(self, veh_id, control_command, obs_dict):
        """Vehicle acts based on the input action.

        Args:
            action (dict): Lonitudinal and lateral actions. It should have the format: {'longitudinal': float, 'lateral': str}. The longitudinal action is the longitudinal acceleration, which should be a float. The lateral action should be the lane change direction. 'central' represents no lane change. 'left' represents left lane change, and 'right' represents right lane change.
        """ 
        # Set Safe Mode from SUMO
        tfl_red_flag = 'tfl_red' in control_command.keys() and control_command['tfl_red'] == 1
        negligence_flag = 'negligence' in control_command.keys() and control_command['negligence'] == 1
        negligence_flag = negligence_flag or tfl_red_flag

        # # disable MOBIL lane change
        # control_command["lateral"] = "central"

        if not self.is_busy:    
            if negligence_flag:
                utils.set_vehicle_speedmode(veh_id, 0)
                if not self.is_busy:
                    self.controlled_duration = self.neg_step_num  # begin counting the negligence timesteps
                    self.is_busy = True
            else:
                # utils.set_vehicle_speedmode(veh_id, 1)
                utils.set_vehicle_speedmode(veh_id)
        
        # Longitudinal control
        if control_command["longitudinal"] == "SUMO":
            utils.set_vehicle_speedmode(veh_id)
            controlled_acc = None
        else:
            controlled_acc = control_command["longitudinal"]
            current_velocity = obs_dict["ego"].data["speed"]
            if current_velocity + controlled_acc > self.params["v_high"]:
                controlled_acc = self.params["v_high"] - current_velocity
            elif current_velocity + controlled_acc < self.params["v_low"]:
                controlled_acc = self.params["v_low"] - current_velocity
        # Lateral control
        # lane_keep_duration = self.params["acc_duration"]
        lane_keep_duration = self.params["neg_duration"] if negligence_flag else self.params["acc_duration"]
        if control_command["lateral"] == "SUMO":
            utils.set_vehicle_lanechangemode(veh_id)
            if controlled_acc:
                self.simulator.change_vehicle_speed(veh_id, controlled_acc, lane_keep_duration)
        else:
            
            if control_command["lateral"] == "central":
                utils.set_vehicle_lanechangemode(veh_id, 512) # disable lane change in central mode
                # current_lane_offset = utils.get_vehicle_lateral_lane_position(veh_id)
                # self.simulator.change_vehicle_sublane_dist(veh_id, -current_lane_offset, self.step_size)
                if controlled_acc:
                    self.simulator.change_vehicle_speed(veh_id, controlled_acc, lane_keep_duration)
            else:
                # utils.set_vehicle_speedmode(veh_id, 1)
                lane_change_mode = 0 if utils.get_distance(veh_id) > 10 else 512
                utils.set_vehicle_lanechangemode(veh_id, lane_change_mode)
                utils.set_vehicle_speedmode(veh_id)
                self.simulator.change_vehicle_lane(veh_id, control_command["lateral"], self.params["lc_duration"])
                if controlled_acc:
                    self.simulator.change_vehicle_speed(veh_id, controlled_acc, self.params["lc_duration"])
                self.controlled_duration = self.lc_step_num + random.randint(5, 10) # begin counting the lane change maneuver timesteps
                self.is_busy = True
                vehicle_edge_id = obs_dict["local"].data["Ego"]["road_id"]
                if "EG_1_1_1" not in vehicle_edge_id and "EG_1_3_1" not in vehicle_edge_id and "NODE_7_0" not in vehicle_edge_id: # vehicle not on highway
                    self.execute_urban_lanechange(veh_id, control_command, obs_dict)