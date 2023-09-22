from .env_monitor import EnvMonitor
from terasim.envs.template import EnvTemplate
import terasim.utils as utils
import numpy as np
from terasim.overlay import traci

class SafeTestNDE(EnvTemplate):

    def __init__(self, vehicle_factory, info_extractor):
        self.warmup_time = np.random.randint(low=15*60, high=20*60) # 15 minutes
        self.run_time = 5*60 # 5 minutes
        super().__init__(vehicle_factory, info_extractor)

    def on_start(self, ctx):
        self.sumo_warmup(self.warmup_time)
        return super().on_start(ctx)
    
    def sumo_warmup(self, warmup_time):
        while True:
            traci.simulationStep()
            if traci.simulation.getTime() > warmup_time:
                break
        self._vehicle_in_env_distance("before")

    def on_step(self, ctx):
        # Make decisions and execute commands
        control_cmds = self.make_decisions()
        self.execute_control_commands(control_cmds)
        self.monitor.add_observation(control_cmds)
        # Simulation stop check
        return self.should_continue_simulation()
    
    def on_stop(self, ctx):
        return super().on_stop(ctx)
    
    def collision_log(self):
        return "0"
    
    def has_negligence(self, veh_1_id, veh_2_id):
        if veh_1_id in self.monitor.negligence_mode.keys():
            return True
        elif veh_2_id in self.monitor.negligence_mode.keys():
            return True
        return False
    
    def _vehicle_in_env_distance(self, mode):
        veh_id_list = traci.vehicle.getIDList()
        distance_dist = self._get_distance(veh_id_list)
        # check point for debugging
        # velocity = sum([traci.vehicle.getSpeed(veh_id) for veh_id in veh_id_list])
        # print("distance: {}, velocity: {}".format(distance, velocity))
        self.monitor.update_distance(distance_dist, mode)
    
    def _get_distance(self, veh_id_list):
        distance_dist = {veh_id: utils.get_distance(veh_id) for veh_id in veh_id_list}
        return distance_dist
    
    def _add_vehicle_to_env(self, veh_id_list): 
        single_input = not isinstance(veh_id_list, list)
        if single_input:
            veh_id_list = [veh_id_list]
        # self.monitor.add_vehicle_to_route(veh_id_list)
        output = super()._add_vehicle_to_env(veh_id_list)
        return output[0] if single_input else output
    
    # def _remove_vehicle_from_env(self, veh_id_list):
    #     if not isinstance(veh_id_list, list):
    #         veh_id_list = [veh_id_list]
    #     if utils.get_time() > self.warmup_time:
    #         distance_dist = self._get_distance(veh_id_list)
    #         self.monitor.update_distance(distance_dist, "after")
    #     # self.monitor.remove_vehicle_records(veh_id_list)
    #     super()._remove_vehicle_from_env(veh_id_list)
    
    def _collision_summary(self, veh_1_id, veh_2_id):
        veh1_mode = utils.get_vehicle_speedmode(veh_1_id)
        veh2_mode = utils.get_vehicle_speedmode(veh_2_id)
        veh1_lanemode = utils.get_vehicle_lanechangemode(veh_1_id)
        veh2_lanemode = utils.get_vehicle_lanechangemode(veh_2_id)
        veh1_angle = utils.get_vehicle_angle(veh_1_id)
        veh2_angle = utils.get_vehicle_angle(veh_2_id)
        veh1_dist = utils.get_distance(veh_1_id)
        veh2_dist = utils.get_distance(veh_2_id)
        print("collision", veh_1_id, veh_2_id, self.collision_log(), utils.get_time(), sep="\t")
        print("collision_mode", veh1_mode, veh2_mode, veh1_lanemode, veh2_lanemode, sep="\t")
        print("collision_dist", veh1_dist, veh2_dist, veh1_angle, veh2_angle, sep="\t")

    def should_continue_simulation(self):
        # stop when collision happens or 300s
        num_colliding_vehicles = self.simulator.get_colliding_vehicle_number()
        self._vehicle_in_env_distance("after")
        if num_colliding_vehicles >= 2:
            colliding_vehicles = self.simulator.get_colliding_vehicles()
            veh_1_id = colliding_vehicles[0]
            veh_2_id = colliding_vehicles[1]
            # if not self.has_negligence(veh_1_id, veh_2_id):
            #     print("ignore_collision", veh_1_id, veh_2_id, self.collision_log(), utils.get_time(), sep="\t")
            #     return True
            # monitor the collision
            self.monitor.save_observation(veh_1_id, veh_2_id)
            self.monitor.export_final_state(veh_1_id, veh_2_id, self.collision_log(), "collision")
            # output the collision info
            # self._collision_summary(veh_1_id, veh_2_id)
            return False
        elif utils.get_time() >= self.warmup_time + self.run_time:
            self.monitor.export_final_state(None, None, self.collision_log(), "timeout")
            # print("timeout", " ", " ", self.collision_log(), utils.get_time(), sep="\t")
            # velocity = sum([traci.vehicle.getSpeed(veh_id) for veh_id in traci.vehicle.getIDList()])
            # print("velocity: {}".format(velocity))
            return False
        return super().should_continue_simulation()

    # def detect_collision(self):
    #     cav_info = self.vehicle_list['CAV'].observation.information
    #     cav_head_pos = cav_info["Ego"]["position"]
    #     cav_heading = cav_info["Ego"]["heading"]/180*math.pi
    #     cav_param = utils.CAR_PARAM()
    #     cav_center_list = utils.three_circle_center_helper(cav_head_pos, cav_heading, cav_param)
    #     num_veh = 0
    #     cav_context_info = self.vehicle_list['CAV'].observation.context
    #     for key in cav_context_info:
    #         if key != "CAV":
    #             num_veh += 1
    #             veh_head_pos = cav_context_info[key][66]
    #             veh_heading = cav_context_info[key][67]/180*math.pi
    #             if "PERSON" in key:
    #                 veh_param = utils.PERSON_PARAM()
    #                 # continue
    #             else:
    #                 veh_param = utils.CAR_PARAM()
    #             veh_center_list = utils.three_circle_center_helper(veh_head_pos, veh_heading, veh_param)
    #             if utils.detect_crash_three_circle(cav_center_list, cav_param, veh_center_list, veh_param):
    #                 if utils.sat_detection(cav_center_list[1], cav_heading, cav_param, veh_center_list[1], veh_heading, veh_param):
    #                     return True, ("CAV",  key)
    #     return False, ()