from .safetest_nde import SafeTestNDE
import sumolib
from terasim.overlay import traci
import terasim.utils as utils
import numpy as np
import math
veh_length = 5.0
veh_width = 2.0
circle_r = 1.3
tem_len = math.sqrt(circle_r**2-(veh_width/2)**2)
from terasim_mr.utils.convertion import sumo_coordinate_to_center_coordinate, sumo_heading_to_orientation

class Point:
    def __init__(self, position_tuple):
        x,y = position_tuple[0], position_tuple[1]
        self.x = x
        self.y = y
    
    def __str__(self) -> str:
        return f"({self.x}, {self.y})"
    
def check_func(PointList):
    for i in range(len(PointList)-1):
        if (PointList[i].x - PointList[i+1].x)**2 + (PointList[i].y - PointList[i+1].y)**2 > 256:
            return False
    return True

def collision_check(traj1, traj2):
    time_series = list(traj1.keys())
    for time in time_series:
        center_list_1 = get_circle_center_list(traj1[time])
        center_list_2 = get_circle_center_list(traj2[time])
        for p1 in center_list_1:
            for p2 in center_list_2:
                dist = cal_dist(p1, p2)
                if dist <= 2*circle_r:
                    return True
    return False

def get_circle_center_list(traj_point):
    center1 = (traj_point["x_lon"], traj_point["x_lat"])
    heading = traj_point["heading"]
    center0 = (
        center1[0]+(veh_length/2-tem_len)*math.cos(heading),
        center1[1]+(veh_length/2-tem_len)*math.sin(heading)
    )
    center2 = (
        center1[0]-(veh_length/2-tem_len)*math.cos(heading),
        center1[1]-(veh_length/2-tem_len)*math.sin(heading)
    )
    center_list = [center0, center1, center2]
    return center_list


def cal_dist(p1, p2):
    x1, y1 = p1[0], p1[1]
    x2, y2 = p2[0], p2[1]
    return math.sqrt((x1-x2)**2+(y1-y2)**2)


class SafeTestNADE(SafeTestNDE):

    def on_start(self, ctx):
        self.importance_sampling_weight = 1.0
        return super().on_start(ctx)

    # @profile
    def on_step(self, ctx):
        self._vehicle_in_env_distance("after")
        # Make decisions and execute commands
        control_cmds, infos = self.make_decisions()
        ITE_control_cmds, weight = self.ITE_decision(control_cmds, infos) # enable ITE
        # ITE_control_cmds = {veh_id: control_cmds[veh_id]["command"] for veh_id in control_cmds} # disable ITE
        # record the negligence mode
        # self.negligence_record(ITE_control_cmds)
        self.monitor.update_negligence_mode(ITE_control_cmds)
        # monitor the environment
        self.monitor.add_observation(ITE_control_cmds)
        # weight = 1.0 # disable ITE
        self.execute_control_commands(ITE_control_cmds)
        self.importance_sampling_weight *= weight # update the importance sampling weight
        # Simulation stop check
        return self.should_continue_simulation()

    def final_state_log(self):
        # return f"weight: {Decimal(self.importance_sampling_weight):.2E}"
        neg_log_importance_sampling_weight = -np.log10(self.importance_sampling_weight)
        original_final_state_log = super().final_state_log()
        original_final_state_log.update({"importance": neg_log_importance_sampling_weight})
        return original_final_state_log
        
    # @profile
    def ITE_decision(self, control_command_dict, control_info_dict):
        """NADE decision here.

        Args:
            control_command_dict (dict): for each vehicle v_id, control_command_dict[veh_id] = control_command, control_info. In nade, control info denotes the ndd pdf of the given BV
        """
        default_ndd_info = {
            "normal":{
                "command": {
                    "lateral": "central",
                    "longitudinal": 0.0,
                    "type": "lon_lat",
                    },
                "prob": 1.0,
                }
            }

        control_command_dict = {
            veh_id: {
                "command": control_command_dict[veh_id],
                "ndd": control_info_dict[veh_id] if control_info_dict[veh_id] is not None else default_ndd_info,
            } for veh_id in control_command_dict
        }
        ndd_dict = {veh_id: control_command_dict[veh_id]["ndd"] for veh_id in control_command_dict}
        trajectory_dict = {veh_id: self.predict_future_trajectory_dict(veh_id, 5, 0.6, ndd_dict[veh_id])[0] for veh_id in control_command_dict}
        maneuver_challenge_dict, maneuver_challenge_info = self.get_maneuver_challenge_dict(trajectory_dict)
        time = utils.get_time()
        self.monitor.add_maneuver_challenges(maneuver_challenge_dict, time)
        criticality_dict = {veh_id: self.get_criticality_dict(ndd_dict[veh_id], maneuver_challenge_dict[veh_id]) for veh_id in control_command_dict}
        
        # TO-DO: return the ITE control command
        ITE_control_command_dict, weight = self.ITE_importance_sampling(control_command_dict, criticality_dict)
        neglect_pair_list = self.get_neglecting_vehicle_id(ITE_control_command_dict, maneuver_challenge_info)
        neglected_vehicle_list = [pair[1] for pair in neglect_pair_list]
        ITE_control_command_dict = self.apply_collision_avoidance(neglected_vehicle_list, ITE_control_command_dict)

        return ITE_control_command_dict, weight
    
    def apply_collision_avoidance(self, neglected_vehicle_list, ITE_control_command_dict):
        for veh_id in neglected_vehicle_list:
            ITE_control_command_dict[veh_id] = {
                "lateral": "central",
                "longitudinal": -9.0,
                "type": "lon_lat",
                "duration": 2.0,
                "mode": "avoid_collision",
            }
        return ITE_control_command_dict

    def get_neglecting_vehicle_id(self, control_command_dict, maneuver_challenge_info):

        neglect_pair_list = []
        for veh_id in control_command_dict:
            control_command = control_command_dict[veh_id]
            if "mode" in control_command: # neglegence happens
                neglecting_vehicle_id = veh_id
                neglected_vehicle_id = list(maneuver_challenge_info[neglecting_vehicle_id].keys())[0]
                neglect_pair_list.append((neglecting_vehicle_id, neglected_vehicle_id))
        return neglect_pair_list
    
    def ITE_importance_sampling(self, ndd_control_command_dict, criticality_dict):
        """Importance sampling for NADE.

        Args:
            ndd_control_command_dict (dict): for each vehicle v_id, ndd_control_command_dict[veh_id] = ndd_control_command, ndd_pdf
            criticality_dict (dict): for each vehicle v_id, criticality_dict[veh_id] = criticality

        Returns:
            weight (float): the importance sampling weight
        """
        weight = 1.0
        # ITE_control_command_dict = {veh_id: ndd_control_command_dict[veh_id]["command"] for veh_id in ndd_control_command_dict}
        ITE_control_command_dict = {veh_id: ndd_control_command_dict[veh_id]["ndd"]["normal"]["command"] for veh_id in ndd_control_command_dict}
        
        default_max_IS_prob = 0.1 # epsilon = 0.95 importance sampling
        for veh_id in criticality_dict:
            if "negligence" in criticality_dict[veh_id] and criticality_dict[veh_id]["negligence"]:
                sampled_prob = np.random.uniform(0, 1)
                ndd_normal_prob = ndd_control_command_dict[veh_id]["ndd"]["normal"]["prob"]
                ndd_negligence_prob = ndd_control_command_dict[veh_id]["ndd"]["negligence"]["prob"]
                assert ndd_normal_prob + ndd_negligence_prob == 1, "The sum of the probabilities of the normal and negligence control commands should be 1."
                # IS_prob = default_max_IS_prob
                IS_prob = np.clip(criticality_dict[veh_id]["negligence"] * 2e3, 0, default_max_IS_prob)
                if sampled_prob < IS_prob: # select the negligece control command
                    weight *= ndd_negligence_prob / IS_prob
                    ITE_control_command_dict[veh_id] = ndd_control_command_dict[veh_id]["ndd"]["negligence"]["command"]
                else:
                    weight *= ndd_normal_prob / (1 - IS_prob)
                    ITE_control_command_dict[veh_id] = ndd_control_command_dict[veh_id]["ndd"]["normal"]["command"]
        return ITE_control_command_dict, weight

    # @profile
    def get_maneuver_challenge_dict(self, trajectory_dict):
        """Get the maneuver challenge for each vehicle when it is in the negligence mode while other vehicles are in the normal mode.

        Args:
            trajectory_dict (dict): trajectory_dict[veh_id] = {"normal": normal_future_trajectory, "negligence": negligence_future_trajectory}

        Returns:
            maneuver_challenge_dict (dict): maneuver_challenge_dict[veh_id] = (num_affected_vehicles, affected_vehicles)
        """
        normal_future_trajectory_dict = {
            veh_id: trajectory_dict[veh_id].get("normal", None) for veh_id in trajectory_dict
        }
        negligence_future_trajectory_dict = {
            veh_id: trajectory_dict[veh_id].get("negligence", None) for veh_id in trajectory_dict
        }
        avoidance_future_trajectory_dict = {
            veh_id: trajectory_dict[veh_id].get("avoid_collision", None) for veh_id in trajectory_dict
        }
        maneuver_challenges = {
            veh_id: self.get_maneuver_challenge(veh_id, negligence_future_trajectory_dict[veh_id], normal_future_trajectory_dict) for veh_id in trajectory_dict
        }
        maneuver_challenge_avoidance_dict = {
            veh_id: self.get_maneuver_challenge(veh_id, negligence_future_trajectory_dict[veh_id], avoidance_future_trajectory_dict, highlight_flag=False) for veh_id in trajectory_dict
        }

        for veh_id in maneuver_challenges:
            if maneuver_challenges[veh_id]["maneuver_challenge"] and maneuver_challenge_avoidance_dict[veh_id]["maneuver_challenge"]:
                maneuver_challenges[veh_id]["maneuver_challenge"] = 0
                maneuver_challenges[veh_id]["info"] = {}

        for veh_id in maneuver_challenges:
            if maneuver_challenges[veh_id]["maneuver_challenge"]:
                utils.highlight_vehicle(veh_id, duration=0.1)
                try:
                    neglected_veh_id = list(maneuver_challenges[veh_id]["info"].keys())[0]
                    utils.highlight_vehicle(neglected_veh_id, duration=0.1)
                except:
                    print(f"no neglected vehicle for {veh_id}")

        maneuver_challenge_info = {}
        # normalize the maneuver challenge
        maneuver_challenge_dict = {veh_id: {"normal": 0} for veh_id in trajectory_dict}
        for veh_id in trajectory_dict:
            if "negligence" in trajectory_dict[veh_id]:
                maneuver_challenge_dict[veh_id]["negligence"] = maneuver_challenges[veh_id]["maneuver_challenge"]
                maneuver_challenge_info[veh_id] = maneuver_challenges[veh_id]["info"]
        return maneuver_challenge_dict, maneuver_challenge_info

    def get_maneuver_challenge_BV_22(self, negligence_veh_id, negligence_veh_future, all_normal_veh_future):
        bv_22_future = {veh_id: all_normal_veh_future[veh_id] for veh_id in all_normal_veh_future if "BV_22." in veh_id}
        return self.get_maneuver_challenge(negligence_veh_id, negligence_veh_future, bv_22_future)

    # @profile
    def get_maneuver_challenge(self, negligence_veh_id, negligence_veh_future, all_normal_veh_future, highlight_flag=True):
        """Get the maneuver challenge for the negligence vehicle.

        Args:
            negligence_veh_id (str): the id of the negligence vehicle
            negligence_veh_future (list): the future trajectory of the negligence vehicle
            all_normal_veh_future (dict): all_normal_veh _future[veh_id] = future trajectory of the normal vehicle

        Returns:
            num_affected_vehicles (int): the number of vehicles that will be affected by the negligence vehicle
            maneuver_challenge_info (dict): maneuver_challenge_info[veh_id] = 1 if the negligence vehicle will affect the normal vehicle
        """
        # see if the one negligence future will intersect with other normal futures
        maneuver_challenge_info = {}
        if negligence_veh_future is not None and all_normal_veh_future is not None:
            for veh_id in all_normal_veh_future:
                if veh_id == negligence_veh_id:
                    continue
                if self.is_intersect(negligence_veh_future, all_normal_veh_future[veh_id]):
                    # if highlight_flag:
                    #     utils.highlight_vehicle(veh_id, duration=2)
                    #     utils.highlight_vehicle(negligence_veh_id, duration=2)
                    maneuver_challenge_info[veh_id] = 1
        return {"maneuver_challenge": int(len(maneuver_challenge_info) > 0), "info": maneuver_challenge_info} # the first element is the number of vehicles that will be affected by the negligence vehicle
    
    # @profile
    def is_intersect(self, trajectory1, trajectory2):
        """Check if two trajectories intersect.

        Args:
            trajectory1 (list): List of points representing trajectory 1.
            trajectory2 (list): List of points representing trajectory 2.

        Returns:
            bool: True if the trajectories intersect, False otherwise.
        """
        if cal_dist(trajectory1["position"][0], trajectory2["position"][0]) > 30:
            return False

        time_steps = min(len(trajectory1["position"]), len(trajectory2["position"]))
        trajectory1_position = [Point(position_tuple) for position_tuple in trajectory1["position"]]
        trajectory2_position = [Point(position_tuple) for position_tuple in trajectory2["position"]]
        # if the initial position of trajectory1 and trajectory2 distance are larger than a threshold, then they are not intersected
        traj_1_dict = {timestep: {
            "x_lon": trajectory1["position"][timestep][0],
            "x_lat": trajectory1["position"][timestep][1],
            "heading": sumo_heading_to_orientation(trajectory1["heading"][timestep]),
        } for timestep in range(time_steps)}
        traj_2_dict = {timestep: {
            "x_lon": trajectory2["position"][timestep][0],
            "x_lat": trajectory2["position"][timestep][1],
            "heading": sumo_heading_to_orientation(trajectory2["heading"][timestep]),
        } for timestep in range(time_steps)}

        # three circle collision check
        # ! the vehicle length thing should not be hard coded
        collision_check_result = collision_check(traj_1_dict, traj_2_dict)
        if collision_check_result:
            return True

        for i in range(time_steps - 1):
            segment1_start = trajectory1_position[i]
            segment1_end = trajectory1_position[i + 1]
            segment2_start = trajectory2_position[i]
            segment2_end = trajectory2_position[i + 1]

            if self.do_segments_intersect(segment1_start, segment1_end, segment2_start, segment2_end):
                return True

        return False
    
    def do_segments_intersect(self, segment1_start, segment1_end, segment2_start, segment2_end):
        """Check if two line segments intersect.

        Args:
            segment1_start (tuple): Start point of segment 1 (x1, y1).
            segment1_end (tuple): End point of segment 1 (x2, y2).
            segment2_start (tuple): Start point of segment 2 (x3, y3).
            segment2_end (tuple): End point of segment 2 (x4, y4).

        Returns:
            bool: True if the segments intersect, False otherwise.
        """
        def orientation(p, q, r):
            # to find the orientation of an ordered triplet (p,q,r)
            # function returns the following values:
            # 0 : Collinear points
            # 1 : Clockwise points
            # 2 : Counterclockwise
            
            # See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/
            # for details of below formula.
            val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y))
            if val == 0:
                # Collinear orientation
                return 0
            elif (val > 0):
                # Clockwise orientation
                return 1
            else:
                # Counterclockwise orientation
                return 2

        def on_segment(p, q, r):
            if ( (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and
                (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))):
                return True
            return False

        # Calculate orientations for all possible cases
        o1 = orientation(segment1_start, segment1_end, segment2_start)
        o2 = orientation(segment1_start, segment1_end, segment2_end)
        o3 = orientation(segment2_start, segment2_end, segment1_start)
        o4 = orientation(segment2_start, segment2_end, segment1_end)

        # General case
        # o1 != o2: segment2_start and segment2_end are on different sides of segment1_start and segment1_end
        # o3 != o4: segment1_start and segment1_end are on different sides of segment2_start and segment2_end
        if o1 != o2 and o3 != o4:
            return True

        # Special cases (collinear segments with endpoint overlap)
        if o1 == 0 and on_segment(segment1_start, segment2_start, segment1_end):
            return True
        if o2 == 0 and on_segment(segment1_start, segment2_end, segment1_end):
            return True
        if o3 == 0 and on_segment(segment2_start, segment1_start, segment2_end):
            return True
        if o4 == 0 and on_segment(segment2_start, segment1_end, segment2_end):
            return True

        return False


    def get_criticality_dict(self, ndd_dict, maneuver_challenge_dict):
        """Get the criticality of the negligence vehicle.

        Args:
            ndd_dict (dict): the ndd pdf of the given BV
            maneuver_challenge_dict (dict): the information of affected vehicles by the negligence vehicle

        Returns:
            criticality_dict (dict): the criticality of the negligence vehicle
        """
        # assert set(ndd_dict.keys()) == set(maneuver_challenge_dict.keys()), "The keys of ndd_dict and maneuver_challenge_dict should be the same."
        criticality_dict = {}
        for maneuver in ndd_dict:
            criticality_dict[maneuver] = ndd_dict[maneuver]["prob"] * maneuver_challenge_dict[maneuver]
        return criticality_dict

    def predict_future_distance(self, velocity, acceleration, duration_list):
        """Predict the future distance of the vehicle.

        Args:
            velocity (float): the velocity of the vehicle
            acceleration (float): the acceleration of the vehicle
            duration_list (list): the list of duration

        Returns:
            future_distance_list (list): the list of future distance
        """
        future_distance_list = [
            velocity * duration + 0.5 * acceleration * duration * duration for duration in duration_list
        ]
        # the lane_position delta list should be non-decreasing and non-negative
        future_distance_list[0] = max(future_distance_list[0], 0)
        for i in range(1, len(future_distance_list)):
            future_distance_list[i] = max(future_distance_list[i], future_distance_list[i-1])
        return future_distance_list
    
    def predict_future_position_heading(self, veh_info, modality, control_command, duration_list):
        """Predict the future position of the vehicle.

        Args:
            veh_info (dict): the information of the vehicle
            modality (str): the modality of the vehicle
            control_command (str): the control command of the vehicle
            duration_list (list): the list of duration

        Returns:
            future_position_list (list): the list of future position
        """
        road_id = veh_info["road_id"]
        future_distance_list = self.predict_future_distance(veh_info["velocity"], control_command["longitudinal"], duration_list)
        current_lane_length = traci.lane.getLength(veh_info["lane_id"])
        current_route_index = veh_info["route_id_list"].index(road_id)

        new_position_list, new_heading_list = [], []
        # get the displacement of the vehicle in each step
        future_distance_list_delta = np.diff(future_distance_list, prepend=0)
        new_lane_position = veh_info["lane_position"]
        for lane_position_delta in future_distance_list_delta:
            new_lane_position += lane_position_delta
            # find the road id and lane_position of the vehicle after the duration
            while (new_lane_position > current_lane_length) and (veh_info["route_id_list"].index(road_id) < len(veh_info["route_id_list"]) - 1):
                current_route_index = veh_info["route_id_list"].index(road_id)
                road_id = veh_info["route_id_list"][current_route_index + 1]
                new_lane_position = new_lane_position - current_lane_length
                current_lane_length = veh_info["route_length_list"][current_route_index+1]

            # find the new lane index of the vehicle
            max_lane_index = traci.edge.getLaneNumber(road_id)
            if control_command["lateral"] == "left":
                new_lane_index = veh_info["lane_index"] + 1
            elif control_command["lateral"] == "right":
                new_lane_index = veh_info["lane_index"] - 1
            else:
                new_lane_index = veh_info["lane_index"]
            new_lane_index = 0 if new_lane_index >= max_lane_index else new_lane_index
            new_lane_id = road_id + f"_{new_lane_index}"

            # clip the new lane_position (why we need this?)
            # if the last simulation position is out of the route
            # but no need, if two vehicles arrive within 3 seconds
            max_new_lane_position = traci.lane.getLength(road_id+f"_{new_lane_index}")
            new_lane_position = np.clip(new_lane_position, 0, max_new_lane_position)
            
            # change the lane position and index to global position
            new_position = traci.simulation.convert2D(road_id, new_lane_position, new_lane_index)
            new_heading = traci.lane.getAngle(new_lane_id, new_lane_position)
            new_position_center = sumo_coordinate_to_center_coordinate(new_position[0], new_position[1], sumo_heading_to_orientation(new_heading), veh_info["length"])
            new_position_list.append(new_position_center)
            new_heading_list.append(new_heading)

            if (new_lane_position == max_new_lane_position) and (veh_info["route_id_list"].index(road_id) == len(veh_info["route_id_list"]) - 1): # the vehicle is at the end of the route
                break
        return {
            "position": new_position_list,
            "heading": new_heading_list,
            "modality": modality,
            "control_command": control_command,
        }

    def predict_future_trajectory_dict(self, veh_id, time_horizon, time_resolution, ndd_decision_dict = None):
        """Predict the future trajectory of the vehicle.

        Args:
            veh_id (str): the id of the vehicle
            time_horizon (float): the time horizon of the prediction
            time_resolution (float): the time resolution of the prediction
            ndd_decision_dict (dict): the decision of the NDD

        Returns:
            future_trajectory_dict (dict): the future trajectory of the vehicle
        """
        veh_info = self.get_vehicle_info(veh_id)
        # include the original position
        duration_list = [time_horizon_id*time_resolution for time_horizon_id in range(time_horizon+1)]
        trajectory_dict = {}
        trajectory_dict["initial"] = {
            "position": veh_info['position'],
            "velocity": veh_info['velocity'],
            "heading": veh_info['heading'],
        }
        avoid_collision_control_command = {
            "lateral": "central",
            "longitudinal": -4.0,
            "type": "lon_lat",
            "duration": 2.0,
        }
        # For each vehicle, predict the future trajectory according to the NDD decision modalities, plus the avoid_collision modality
        for modality in ndd_decision_dict:
            execute_modality = "normal" if veh_id == "CAV" else modality
            control_command = ndd_decision_dict[execute_modality]
            trajectory_dict[modality] = self.predict_future_position_heading(veh_info, execute_modality, control_command["command"], duration_list)
        trajectory_dict["avoid_collision"] = self.predict_future_position_heading(veh_info, "avoid_collision", avoid_collision_control_command, duration_list)
        return trajectory_dict, veh_info, duration_list

    def get_vehicle_info(self, veh_id):
        """Generate vehicle information for future trajectory prediction

        Args:
            veh_id (str): input vehicle id

        Returns:
            veh_info (dict): output dictionary of vehicle information
        """
        veh_info = {
            "id": veh_id,
            "route": traci.vehicle.getRoute(veh_id),
            "route_index": traci.vehicle.getRouteIndex(veh_id),
            "road_id": traci.vehicle.getRoadID(veh_id),
            "lane_id": traci.vehicle.getLaneID(veh_id),
            "lane_index": traci.vehicle.getLaneIndex(veh_id),
            "position": traci.vehicle.getPosition(veh_id),
            "velocity": traci.vehicle.getSpeed(veh_id),
            "heading": traci.vehicle.getAngle(veh_id),
            "lane_position": traci.vehicle.getLanePosition(veh_id),
            "length": traci.vehicle.getLength(veh_id),
        }
        route_with_internal = sumolib.route.addInternal(self.simulator.sumo_net, veh_info['route'])
        veh_info["route_id_list"] = [route._id for route in route_with_internal]
        veh_info["route_length_list"] = [route._length for route in route_with_internal]
        return veh_info