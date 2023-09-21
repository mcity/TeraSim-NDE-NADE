import json
import math
import redis
import yaml

import terasim-mr.communicationtools.constants as constants
from terasim.overlay import traci
import terasim-mr.utils.others as others
from terasim.vehicle.sensors.base import BaseSensor
from terasim-mr.utils.convertion import utm_to_sumo_coordinate, sumo_to_utm_coordinate, center_coordinate_to_sumo_coordinate, sumo_coordinate_to_center_coordinate, sumo_heading_to_orientation, orientation_to_sumo_heading

class MRTLSSensorLocal(BaseSensor):
    def __init__(self, name="ROSTLS", **params):
        super().__init__(name, **params)
        # load yaml file, will be removed after support for Autoware.ai is removed
        with open(self.params.sumo_tls_addition_info_file_path, 'r') as stream:
            self.autoware_ai_tls_configs = yaml.load(stream, Loader=yaml.FullLoader)
        self.cav_front_tls_info = {}
        self.tls_id_list = []
        self.tls_config_dict = {}
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        self.redis_key = constants.REDIS_CONSTANTS.CAV_FRONT_TLS_INFO_ROS

    def fetch(self):
        self.update_tls_info()
        self.upload_tls_info()
        return self.cav_front_tls_info
    
    def get_tls_id_list(self):
        """
        Get traffic light signal id list
        """
        return traci.trafficlight.getIDList()
    
    def get_tls_utm_coordinate(self, tls_id):
        """
        Get traffic light signal position
        """
        return [0.0, 0.0]
        # return sumo_to_utm_coordinate(traci.trafficlight.getPosition(tls_id))
    
    def get_all_tls_info(self):
        """Get all traffic light signal information for the whole simulation (or around CAV)"""
        # pre-load tls_id_list and tls_config_dict
        self.tls_id_list = self.get_tls_id_list() if self.tls_id_list == [] else self.tls_id_list
        self.tls_config_dict = {tls_id: {"utm_coordinate": self.get_tls_utm_coordinate(tls_id)} for tls_id in self.tls_id_list} if self.tls_config_dict == {} else self.tls_config_dict

        # get all tls info, including state and position
        tls_info = {}
        for tls_id in self.tls_id_list:
            tls_info[tls_id] = {}
            tls_info[tls_id]["tls_state"] = traci.trafficlight.getRedYellowGreenState(tls_id)
            tls_info[tls_id]["utm_coordinate"] = self.tls_config_dict[tls_id]["utm_coordinate"]
        return tls_info
    
    def update_tls_info(self):
        """Update traffic light signal information for CAV
        """
        vehID = self._agent.id
        lane_id = traci.vehicle.getLaneID(vehID)
        print("CAV", lane_id)
        front_tls_status = 0
        stop_find_flag = False
        if self.autoware_ai_tls_configs["signal_entry_lane"] == {}: # if no traffic light, stop finding traffic lights
            stop_find_flag = True
        cv_ahead_flag = False
        new_tls_id = 0
        stopline1_x, stopline1_y = -100, -100
        stopline2_x, stopline2_y = -100, -100

        if not stop_find_flag:
            for tls_id in self.autoware_ai_tls_configs["signal_id_list"]:
                for i in range(len(self.autoware_ai_tls_configs["signal_entry_lane"][tls_id]["lane"])):
                    lane_list = self.autoware_ai_tls_configs["signal_entry_lane"][tls_id]["lane"][i]
                    if lane_id in lane_list:
                        tls_info = traci.trafficlight.getRedYellowGreenState(tls_id)
                        front_tls_status = tls_info[i]
                        leader_info = traci.vehicle.getLeader("CAV", 30) # empty leader: None
                        if leader_info is not None and traci.vehicle.getLaneID(leader_info[0]) in lane_list:
                            leader_head_center = traci.vehicle.getPosition(leader_info[0])
                            leader_heading = traci.vehicle.getAngle(leader_info[0])/180*math.pi
                            veh_length = 4.8
                            if "CV" in leader_info[0]:
                                veh_length += traci.vehicle.getSpeed("CAV")+2
                                cv_ahead_flag = True
                            leader_rear_left, leader_rear_right = others._cal_rear_points(leader_head_center, leader_heading, length=veh_length)
                            stopline1_x, stopline1_y = leader_rear_right
                            stopline2_x, stopline2_y = leader_rear_left
                            print("!!!!!!!!!!Distance to effective leader:",leader_info, traci.vehicle.getMinGap("CAV"))
                        else:
                            if leader_info is not None:
                                print("??????????????Distance to ineffective leader:",leader_info, traci.vehicle.getMinGap("CAV"))
                                print("Reason",traci.vehicle.getRoadID(leader_info[0]),lane_list)
                            orien_ = self.autoware_ai_tls_configs["signal_entry_lane"][tls_id]["orientation"][i]
                            stopline1_x, stopline1_y = self.autoware_ai_tls_configs["signal_entry_lane"][tls_id]["stoplines"][orien_][0]
                            stopline2_x, stopline2_y = self.autoware_ai_tls_configs["signal_entry_lane"][tls_id]["stoplines"][orien_][1]
                        stop_find_flag = True
                        break
                if stop_find_flag:
                    break

        if front_tls_status != 0:
            front_tls_status = int(front_tls_status in ["G","g"])+1
            if cv_ahead_flag:
                front_tls_status = 2
            # front_tls_status = 1
            new_tls_id = self.autoware_ai_tls_configs["signal_entry_lane"][tls_id]["new_index"]
        self.cav_front_tls_info = {
            "front_tls_status": front_tls_status,
            "new_tls_id": new_tls_id,
            "stopline1_x": stopline1_x,
            "stopline1_y": stopline1_y,
            "stopline2_x": stopline2_x,
            "stopline2_y": stopline2_y,
            "tls_info": self.get_all_tls_info(),
        }

    def upload_tls_info(self):
        """upload traffic light signal information to Redis server (the key is "cav_front_tls_info")
        """
        str_cav_front_tls_info = json.dumps(self.cav_front_tls_info)
        self.redis_client.set(self.redis_key, str_cav_front_tls_info)


class MRTLSSensorRemote(MRTLSSensorLocal):
    def __init__(self, name="WebTLS", **params):
        super().__init__(name, **params)
        self.redis_key = constants.REDIS_CONSTANTS.CAV_FRONT_TLS_INFO_WEB_PUB
