import redis
import time

from terasim.overlay import traci
from .safetest_nade_with_av import SafeTestNADEWithAV
import terasim.utils as utils


REDIS_CONSTANTS_TERASIM_STATE = "terasim_state"
REDIS_CONSTANTS_AUTOWARE = "autoware_state"


class SafeTestNADECoSim(SafeTestNADEWithAV):

    def __init__(self, vehicle_factory, info_extractor):
        super().__init__(vehicle_factory, info_extractor)
        self.warmup_time = 15*60 # 15 minutes
        self.run_time = 2*60 # 2 minutes

    def on_start(self, ctx):
        super().on_start(ctx)
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        self.redis_client.set(REDIS_CONSTANTS_TERASIM_STATE, "1")
        time.sleep(5)
    
    def should_continue_simulation(self):
        # stop when collision happens or 300s
        collision_id_list = traci.simulation.getCollidingVehiclesIDList()
        num_colliding_vehicles = self.simulator.get_colliding_vehicle_number()
        if num_colliding_vehicles >= 2 and "CAV" in collision_id_list:
            self._vehicle_in_env_distance("after")
            colliding_vehicles = self.simulator.get_colliding_vehicles()
            veh_1_id = colliding_vehicles[0]
            veh_2_id = colliding_vehicles[1]
            self.monitor.save_observation(veh_1_id, veh_2_id)
            self.monitor.export_final_state(veh_1_id, veh_2_id, self.collision_log(), "collision")
            # output the collision info
            # self._collision_summary(veh_1_id, veh_2_id)
            return False
        else:
            cav_edge_id = traci.vehicle.getRoadID("CAV")
            if cav_edge_id == "EG_29_1_1":
                self._vehicle_in_env_distance("after")
                self.monitor.export_final_state(None, None, self.collision_log(), "reachend")
                # print("timeout", " ", " ", self.collision_log(), utils.get_time(), sep="\t")
                return False
            elif utils.get_time() >= self.warmup_time + self.run_time:
                self.monitor.export_final_state(None, None, self.collision_log(), "timeout")
                # print("timeout", " ", " ", self.collision_log(), utils.get_time(), sep="\t")
                # velocity = sum([traci.vehicle.getSpeed(veh_id) for veh_id in traci.vehicle.getIDList()])
                # print("velocity: {}".format(velocity))
                return False
        return True

    def on_stop(self, ctx):
        self.redis_client.set(REDIS_CONSTANTS_TERASIM_STATE, "0")
        super().on_stop(ctx)
