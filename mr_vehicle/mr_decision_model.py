import json
import math
import redis

import terasim_mr.communicationtools.constants as constants
from terasim.vehicle.decision_models.base_decision_model import BaseDecisionModel
import terasim.utils as utils
from terasim_mr.utils.convertion import utm_to_sumo_coordinate, sumo_to_utm_coordinate, center_coordinate_to_sumo_coordinate, sumo_coordinate_to_center_coordinate, sumo_heading_to_orientation, orientation_to_sumo_heading

class MRDecisionModelLocal(BaseDecisionModel):
    def __init__(self):
        super().__init__()
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        self.av_state_key = constants.REDIS_CONSTANTS.AV_STATE

    def install(self):
        # utils.set_vehicle_speedmode(self.vehicle.id, 7)
        utils.set_vehicle_speedmode(self.vehicle.id)
        utils.set_vehicle_lanechangemode(self.vehicle.id)
        self.vehicle.simulator.set_vehicle_color(self.vehicle.id, self.vehicle.COLOR_RED) 
    
    def get_av_planning_time(self):
        av_planning_time = self.redis_client.get(constants.REDIS_CONSTANTS.AV_PLANNING_TIME)
        if av_planning_time is not None:
            return av_planning_time.decode()
        else:
            return None

    def derive_control_command_from_observation(self, obs_dict):
        """derive control command from observation

        Args:
            obs_dict (dict): vehicle observation dictionary

        Returns:
            dict: command
        """
        av_planning_time = self.get_av_planning_time()
        
        # while True:
        #     av_planning_time = self.get_av_planning_time()
        #     if av_planning_time is None or av_planning_time == str(utils.get_time()):
        #         # print("Recieve av planning result:", av_planning_time, utils.get_time())
        #         break
            
        av_state = self.redis_client.get(self.av_state_key)
        if av_state is not None:
            # when CAV has input from Redis server (the source is ROS-based Autoware)
            av_state = json.loads(av_state.decode())
            utm_center_coordinates = [av_state['x'], av_state['y']]
            sumo_center_coordinates = utm_to_sumo_coordinate(utm_center_coordinates)
            sumo_front_bumper_coordinates = center_coordinate_to_sumo_coordinate(sumo_center_coordinates[0], sumo_center_coordinates[1], av_state['orientation'])
            sumo_heading = orientation_to_sumo_heading(av_state['orientation'])
            command = {
                "type": "SetSumoTransform",
                "position": (sumo_front_bumper_coordinates[0], sumo_front_bumper_coordinates[1]), # x, y
                "velocity": av_state["velocity"], # m/s
                "angle": sumo_heading, # degree
                "keepRoute": 0
            }
            # print("terasim_control_command", latest_ego_positionheading['x'], latest_ego_positionheading['y'])
            return command, None
        else:
            # otherwise, CAV will be controlled by SUMO
            return None, None
