from .env_monitor import EnvMonitor
from terasim.envs.template import EnvTemplate
import terasim.utils as utils
import numpy as np
from terasim.overlay import traci

class SafeTestNDE(EnvTemplate):

    def __init__(self, vehicle_factory, info_extractor, warmup_time_lb=900, warmup_time_ub=1200, run_time=720):
        rng = np.random.default_rng()
        self.warmup_time = int(rng.integers(low=warmup_time_lb, high=warmup_time_ub)) # 12 minutes
        self.run_time = run_time # 5 minutes
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
    
    def final_state_log(self):
        return {
            "warmup_time": self.warmup_time,
            "run_time": self.run_time,
        }
    
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
    
    def _collision_summary(self, veh_1_id, veh_2_id):
        veh1_mode = utils.get_vehicle_speedmode(veh_1_id)
        veh2_mode = utils.get_vehicle_speedmode(veh_2_id)
        veh1_lanemode = utils.get_vehicle_lanechangemode(veh_1_id)
        veh2_lanemode = utils.get_vehicle_lanechangemode(veh_2_id)
        veh1_angle = utils.get_vehicle_angle(veh_1_id)
        veh2_angle = utils.get_vehicle_angle(veh_2_id)
        veh1_dist = utils.get_distance(veh_1_id)
        veh2_dist = utils.get_distance(veh_2_id)
        print("collision", veh_1_id, veh_2_id, self.final_state_log(), utils.get_time(), sep="\t")
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
            self.monitor.save_observation(veh_1_id, veh_2_id)
            self.monitor.export_final_state(veh_1_id, veh_2_id, self.final_state_log(), "collision")
            return False
        elif utils.get_time() >= self.warmup_time + self.run_time:
            self.monitor.export_final_state(None, None, self.final_state_log(), "timeout")
            return False
        return super().should_continue_simulation()