from terasim.envs.template import EnvTemplate
import terasim.utils as utils
import numpy as np
from terasim.overlay import traci

class SafeTestNDE(EnvTemplate):

    def __init__(self, vehicle_factory, info_extractor, warmup_time_lb=900, warmup_time_ub=1200, run_time=300, *args, **kwargs):
        rng = np.random.default_rng()
        self.warmup_time = int(rng.integers(low=warmup_time_lb, high=warmup_time_ub)) # 12 minutes
        self.run_time = run_time # 5 minutes
        print("warmup_time", self.warmup_time, "run_time", self.run_time)
        super().__init__(vehicle_factory, info_extractor, *args, **kwargs)

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
        traci.simulation.executeMove()
        self._maintain_all_vehicles(ctx)
        control_cmds, infos = self.make_decisions(ctx)
        self.execute_control_commands(control_cmds)
        # self.monitor.add_observation(control_cmds)
        return self.should_continue_simulation()

    def final_state_log(self):
        return {
            "warmup_time": self.warmup_time,
            "run_time": self.run_time,
        }
    
    def _vehicle_in_env_distance(self, mode):
        veh_id_list = traci.vehicle.getIDList()
        distance_dist = self._get_distance(veh_id_list)
        self.monitor.update_distance(distance_dist, mode)
    
    def _get_distance(self, veh_id_list):
        distance_dist = {veh_id: utils.get_distance(veh_id) for veh_id in veh_id_list}
        return distance_dist

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