import json
import numpy as np
import redis
import time
import utm
import terasim.utils as utils
import terasim_mr.communicationtools.constants as constants
from terasim.overlay import traci
from terasim.vehicle.sensors.base import BaseSensor
from terasim_mr.utils.convertion import utm_to_sumo_coordinate, sumo_to_utm_coordinate, center_coordinate_to_sumo_coordinate, sumo_coordinate_to_center_coordinate, sumo_heading_to_orientation, orientation_to_sumo_heading


class MRContextSensorLocal(BaseSensor):
    """Convert the SUMO background vehicle information to the format of UTM and save to Redis.

    Args:
        BaseSensor (_type_): _description_

    Returns:
        _type_: _description_
    """    
    history_length = 10
    max_bvs_num = 20
    info_length = 7
    observation_range = 50
    step_size = 0.1
    x_offset_SUMO, y_offset_SUMO, z_offset_SUMO = 277600-102.89, 4686800-281.25, 0

    def __init__(self, name="ROSContext", **params):
        super().__init__(name, **params)
        self.shared_info_index = 0
        self.time_list = [0.0] * self.history_length
        self.bv_history = [0.0] * (self.history_length * self.max_bvs_num * self.info_length)
        self.context_vehicle_info = {}
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        self.context_vehicle_info_redis_key = constants.REDIS_CONSTANTS.AV_CONTEXT

    def fetch(self):
        self.update_context_vehicle_info()
        self.upload_context_vehicle_info()
        return self.context_vehicle_info
    
    def update_context_vehicle_info(self):
        """update background vehicle information for CAV.
        """
        interested_time_list = [time.time()]
        vehID = self._agent.id
        leading_vehicle_id, leading_vehicle_distance = traci.vehicle.getLeader(vehID, 1000)
        realtime_vehicle_id_list = traci.vehicle.getIDList()
        if vehID in realtime_vehicle_id_list:
            sumo_time = traci.simulation.getTime()
            
            # filter vehicles within observation range
            interested_veh_id_list = [vehID]
            ego_pose = traci.vehicle.getPosition3D(vehID)
            for veh_id in realtime_vehicle_id_list:
                if veh_id != vehID:
                    object_pose = traci.vehicle.getPosition3D(veh_id)
                    dist_ = ((ego_pose[0] - object_pose[0]) ** 2 + (ego_pose[1] - object_pose[1]) ** 2) ** 0.5
                    if dist_ < self.observation_range:
                        interested_veh_id_list.append(veh_id)

            for veh_id in interested_veh_id_list:
                obs = {
                    "acc": traci.vehicle.getAcceleration(veh_id),
                    "orientation": sumo_heading_to_orientation(traci.vehicle.getAngle(veh_id)),
                    "pose": list(traci.vehicle.getPosition3D(veh_id)), # center of front bumper
                    "size": [traci.vehicle.getLength(veh_id),traci.vehicle.getWidth(veh_id),traci.vehicle.getHeight(veh_id)],
                    "slope": traci.vehicle.getSlope(veh_id),
                    "v_lat": traci.vehicle.getLateralSpeed(veh_id),
                    "v_long": traci.vehicle.getSpeed(veh_id),
                    "edge_id": traci.vehicle.getRoadID(veh_id),
                    "lane_id": traci.vehicle.getLaneID(veh_id),
                }
                # convert from center of front bumber to center of mass
                sumo_front_bumper_coordinate = obs["pose"]
                sumo_center_coordinate = sumo_coordinate_to_center_coordinate(sumo_front_bumper_coordinate[0], sumo_front_bumper_coordinate[1], obs["orientation"], obs["size"][0])
                # convert from SUMO coordinate to UTM coordinate
                utm_center_coordinate = sumo_to_utm_coordinate(sumo_center_coordinate)

                lat, lon = utm.to_latlon(utm_center_coordinate[0], utm_center_coordinate[1], 17, 'U')
                if veh_id not in self.context_vehicle_info.keys():
                    angular_speed = 0
                    angular_acc = 0
                    acc_lat = 0
                else:
                    prev_orientation = self.context_vehicle_info[veh_id]["orientation"]
                    orientation = obs["orientation"]
                    prev_angular_speed = self.context_vehicle_info[veh_id]["angular_speed"]
                    prev_v_lat = self.context_vehicle_info[veh_id]["accel_lat"]
                    v_lat = obs["v_lat"]
                    angular_speed = (orientation-prev_orientation)/self.step_size
                    angular_acc = (angular_speed-prev_angular_speed)/self.step_size
                    acc_lat = (v_lat-prev_v_lat)/self.step_size
                
                self.context_vehicle_info[veh_id] = {
                    'x': utm_center_coordinate[0], 'y': utm_center_coordinate[1], 'z': obs["pose"][2],
                    'longitude': lon, 'latitude': lat,
                    'length': obs["size"][0], 'width': obs["size"][1], 'height': obs["size"][2],
                    'orientation': obs["orientation"], 'slope': obs["slope"],
                    'angular_speed': angular_speed, 'angular_acc': angular_acc,
                    'speed_long': obs["v_long"], 'speed_lat': obs['v_lat'],
                    'accel_long': obs["acc"], 'accel_lat': acc_lat,
                    'edge_id': obs["edge_id"], 'lane_id': obs["lane_id"],
                    'bv_history': None,
                    'time': self.time_list,
                    'sim_time': sumo_time
                    "leading_info": {
                        "is_leading_cav": True,
                        "distance": leading_vehicle_distance,
                    } if veh_id == leading_vehicle_id else {
                        "is_leading_cav": False,
                        "distance": None,
                    }
                }
            veh_id_list = list(self.context_vehicle_info.keys())
            for veh_id in veh_id_list:
                if veh_id not in interested_veh_id_list:
                    self.context_vehicle_info.pop(veh_id)
        else:
            veh_id_list = list(self.context_vehicle_info.keys())
            for veh_id in veh_id_list:
                self.context_vehicle_info.pop(veh_id)
        self.shared_info_index = (self.shared_info_index + 1) % self.history_length

    def upload_context_vehicle_info(self):
        """upload context vehicle information to Redis server (the key is "cav_context_vehicle_info")
        """
        str_context_vehicle_info = json.dumps(self.context_vehicle_info)
        self.redis_client.set(self.context_vehicle_info_redis_key, str_context_vehicle_info)
        self.redis_client.set(constants.REDIS_CONSTANTS.TERASIM_TIME, str(utils.get_time()))
