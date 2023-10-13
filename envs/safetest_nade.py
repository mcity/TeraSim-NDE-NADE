from .safetest_nde import SafeTestNDE
import sumolib
from terasim.overlay import traci
import terasim.utils as utils
import numpy as np
from decimal import Decimal

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

class SafeTestNADE(SafeTestNDE):

    def on_start(self, ctx):
        self.importance_sampling_weight = 1.0
        return super().on_start(ctx)

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
        trajectory_dict = {veh_id: self.predict_future_trajectory_dict(veh_id, 5, 0.6, ndd_dict[veh_id]) for veh_id in control_command_dict}
        maneuver_challenge_dict = self.get_maneuver_challenge_dict(trajectory_dict)
        time = utils.get_time()
        self.monitor.add_maneuver_challenges(maneuver_challenge_dict, time)
        criticality_dict = {veh_id: self.get_criticality_dict(ndd_dict[veh_id], maneuver_challenge_dict[veh_id]) for veh_id in control_command_dict}
        
        # TO-DO: return the ITE control command
        ITE_control_command_dict, weight = self.ITE_importance_sampling(control_command_dict, criticality_dict)
        return ITE_control_command_dict, weight
    
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
        
        default_max_IS_prob = 5e-3 # epsilon = 0.95 importance sampling
        for veh_id in criticality_dict:
            if "negligence" in criticality_dict[veh_id] and criticality_dict[veh_id]["negligence"]:
                sampled_prob = np.random.uniform(0, 1)
                ndd_normal_prob = ndd_control_command_dict[veh_id]["ndd"]["normal"]["prob"]
                ndd_negligence_prob = ndd_control_command_dict[veh_id]["ndd"]["negligence"]["prob"]
                assert ndd_normal_prob + ndd_negligence_prob == 1, "The sum of the probabilities of the normal and negligence control commands should be 1."
                # IS_prob = default_IS_prob
                IS_prob = np.clip(criticality_dict[veh_id]["negligence"] * 2e3, 0, default_max_IS_prob)
                if sampled_prob < IS_prob: # select the negligece control command
                    weight *= ndd_negligence_prob / IS_prob
                    ITE_control_command_dict[veh_id] = ndd_control_command_dict[veh_id]["ndd"]["negligence"]["command"]
                else:
                    weight *= ndd_normal_prob / (1 - IS_prob)
                    ITE_control_command_dict[veh_id] = ndd_control_command_dict[veh_id]["ndd"]["normal"]["command"]
        return ITE_control_command_dict, weight

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
        maneuver_challenges = {
            veh_id: self.get_maneuver_challenge(veh_id, negligence_future_trajectory_dict[veh_id], normal_future_trajectory_dict) for veh_id in trajectory_dict
        }

        # normalize the maneuver challenge
        maneuver_challenge_dict = {veh_id: {"normal": 0} for veh_id in trajectory_dict}
        for veh_id in trajectory_dict:
            if "negligence" in trajectory_dict[veh_id]:
                maneuver_challenge_dict[veh_id]["negligence"] = maneuver_challenges[veh_id]["has_occur"]

        return maneuver_challenge_dict

    def get_maneuver_challenge_BV_22(self, negligence_veh_id, negligence_veh_future, all_normal_veh_future):
        bv_22_future = {veh_id: all_normal_veh_future[veh_id] for veh_id in all_normal_veh_future if "BV_22." in veh_id}
        return self.get_maneuver_challenge(negligence_veh_id, negligence_veh_future, bv_22_future)

    def get_maneuver_challenge(self, negligence_veh_id, negligence_veh_future, all_normal_veh_future):
        """Get the maneuver challenge for the negligence vehicle.

        Args:
            negligence_veh_id (str): the id of the negligence vehicle
            negligence_veh_future (list): the future trajectory of the negligence vehicle
            all_normal_veh_future (dict): all_normal_veh_future[veh_id] = future trajectory of the normal vehicle

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
                    utils.highlight_vehicle(veh_id, 2)
                    utils.highlight_vehicle(negligence_veh_id, 2)
                    # print("intersection", negligence_veh_id, veh_id, " ", utils.get_time(), sep="\t")
                    try:
                        assert check_func(negligence_veh_future)
                        assert check_func(all_normal_veh_future[veh_id])
                    except:
                        print("intersection_err", negligence_veh_id, veh_id, " ", utils.get_time(), sep="\t")
                    maneuver_challenge_info[veh_id] = 1
        return {"has_occur": int(len(maneuver_challenge_info) > 0), "info": maneuver_challenge_info} # the first element is the number of vehicles that will be affected by the negligence vehicle
    
    def is_intersect(self, trajectory1, trajectory2):
        """Check if two trajectories intersect.

        Args:
            trajectory1 (list): List of points representing trajectory 1.
            trajectory2 (list): List of points representing trajectory 2.

        Returns:
            bool: True if the trajectories intersect, False otherwise.
        """
        time_steps = min(len(trajectory1), len(trajectory2))
        for i in range(time_steps - 1):
            segment1_start = trajectory1[i]
            segment1_end = trajectory1[i + 1]
            segment2_start = trajectory2[i]
            segment2_end = trajectory2[i + 1]

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
        assert set(ndd_dict.keys()) == set(maneuver_challenge_dict.keys()), "The keys of ndd_dict and maneuver_challenge_dict should be the same."
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
        # the offset delta list should be non-decreasing and non-negative
        future_distance_list[0] = max(future_distance_list[0], 0)
        for i in range(1, len(future_distance_list)):
            future_distance_list[i] = max(future_distance_list[i], future_distance_list[i-1])
        return future_distance_list
    
    def predict_future_position(self, veh_info, modality, control_command, duration_list):
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
        offset_delta_list = self.predict_future_distance(veh_info["velocity"], control_command["longitudinal"], duration_list)
        new_offset = veh_info["offset"] + offset_delta_list[-1] # the vehicle will be at this offset after the duration from the beginning of the road
        current_lane_length = traci.lane.getLength(veh_info["lane_id"])
        current_route_index = veh_info["route_id_list"].index(road_id)

        new_position_list = []
        # get the displacement of the vehicle in each step
        offset_delta_list_delta = np.diff(offset_delta_list, prepend=0)
        new_offset = veh_info["offset"]
        for offset_delta in offset_delta_list_delta:
            new_offset += offset_delta
            # find the road id and offset of the vehicle after the duration
            while (new_offset > current_lane_length) \
                and (veh_info["route_id_list"].index(road_id) < len(veh_info["route_id_list"]) - 1):
                # current_route_index = min(veh_info["route_id_list"].index(road_id), len(veh_info["route_id_list"])-2)
                current_route_index = veh_info["route_id_list"].index(road_id)
                road_id = veh_info["route_id_list"][current_route_index + 1]
                new_offset = new_offset - current_lane_length
                # current_lane_length = veh_info["route_length_list"][veh_info["route_index"]+1]
                current_lane_length = veh_info["route_length_list"][current_route_index+1]

            # find the new lane index of the vehicle
            max_laneIndex = traci.edge.getLaneNumber(road_id)
            if control_command["lateral"] == "left":
                new_laneIndex = veh_info["lane_index"] + 1
            elif control_command["lateral"] == "right":
                new_laneIndex = veh_info["lane_index"] - 1
            else:
                new_laneIndex = veh_info["lane_index"]
            new_laneIndex = 0 if new_laneIndex >= max_laneIndex else new_laneIndex

            # clip the new offset (why we need this?)
            # if the last simulation position is out of the route
            # but no need, if two vehicles arrive within 3 seconds
            max_new_offset = traci.lane.getLength(road_id+f"_{new_laneIndex}")
            new_offset = np.clip(new_offset, 0, max_new_offset)
            
            # change the lane position and index to global position
            new_position = traci.simulation.convert2D(road_id, new_offset, new_laneIndex)
            new_position_list.append(Point(new_position))

            if (new_offset == max_new_offset) and (veh_info["route_id_list"].index(road_id) == len(veh_info["route_id_list"]) - 1):
                break
        return new_position_list

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
        # load required data from traci
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
            "offset": traci.vehicle.getLanePosition(veh_id),
        }

        # self.net not exist
        route_with_internal = sumolib.route.addInternal(self.simulator.sumo_net, veh_info['route'])
        veh_info["route_id_list"] = [route._id for route in route_with_internal]
        veh_info["route_length_list"] = [route._length for route in route_with_internal]

        # include the original position
        duration_list = [time_horizon_id*time_resolution for time_horizon_id in range(time_horizon+1)]
        trajectory_dict = {}
        trajectory_dict["initial"] = {
            "position": veh_info['position'],
            "velocity": veh_info['velocity'],
            "heading": veh_info['heading'],
        }
        for modality in ndd_decision_dict:
            execute_modality = "normal" if veh_id == "CAV" else modality # CAV will only give normal future prediction, while BVs will have both normal and future
            control_command = ndd_decision_dict[execute_modality]
            trajectory_dict[modality] = self.predict_future_position(veh_info, execute_modality, control_command["command"], duration_list)
        return trajectory_dict
