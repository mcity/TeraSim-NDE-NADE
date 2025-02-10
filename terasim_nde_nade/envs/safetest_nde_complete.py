from addict import Dict
from collections import deque
import json
from loguru import logger
import numpy as np
import os

from terasim.envs.template_complete import EnvTemplateComplete
from terasim.overlay import traci
import terasim.utils as utils
from terasim.params import AgentType


class SafeTestNDEComplete(EnvTemplateComplete):

    def __init__(
        self,
        vehicle_factory,
        vru_factory,
        info_extractor,
        warmup_time_lb=900,
        warmup_time_ub=1200,
        run_time=300,
        log_flag=False,
        log_dir=None,
        *args,
        **kwargs,
    ):
        rng = np.random.default_rng()
        self.warmup_time = int(rng.integers(low=warmup_time_lb, high=warmup_time_ub))
        self.run_time = run_time
        logger.info(f"warmup_time: {self.warmup_time}, run_time: {self.run_time}")
        self.final_log = None
        self.log_dir = log_dir
        if self.log_dir:
            os.makedirs(self.log_dir, exist_ok=True)
        self.log_flag = log_flag
        self.tls_info_cache = {}
        self.history_length = 10
        self.record = Dict()
        self.step_epsilon = 1.0
        self.step_weight = 1.0
        super().__init__(vehicle_factory, vru_factory, info_extractor, *args, **kwargs)

    def on_start(self, ctx):
        self.sumo_warmup(self.warmup_time)
        return super().on_start(ctx)

    def get_observation_dicts(self):
        obs_dicts = {
            AgentType.VEHICLE: {
                vehicle.id: vehicle.observation for vehicle in self.vehicle_list.values()
            },
            AgentType.VULNERABLE_ROAD_USER: {
                vru.id: vru.observation for vru in self.vulnerable_road_user_list.values()
            },
        }
        return obs_dicts

    def executeMove(self, ctx, control_cmds=None, ctx_dicts=None, obs_dicts=None):
        for veh_id in traci.vehicle.getIDList():
            traci.vehicle.setSpeed(veh_id, -1)
        traci.simulation.executeMove()
        self._maintain_all_vehicles(ctx)
        self._maintain_all_vulnerable_road_users(ctx)
        existing_vehicle_list = traci.vehicle.getIDList()
        existing_vru_list = traci.person.getIDList()

        control_cmds = {
            AgentType.VEHICLE: (
                {
                    veh_id: control_cmds[AgentType.VEHICLE][veh_id]
                    for veh_id in control_cmds[AgentType.VEHICLE]
                    if veh_id in existing_vehicle_list
                }
                if control_cmds[AgentType.VEHICLE]
                else {}
            ),
            AgentType.VULNERABLE_ROAD_USER: (
                {
                    vru_id: control_cmds[AgentType.VULNERABLE_ROAD_USER][vru_id]
                    for vru_id in control_cmds[AgentType.VULNERABLE_ROAD_USER]
                    if vru_id in existing_vru_list
                }
                if control_cmds[AgentType.VULNERABLE_ROAD_USER]
                else {}
            ),
        }
        obs_dicts = {
            AgentType.VEHICLE: (
                {
                    veh_id: obs_dicts[AgentType.VEHICLE][veh_id]
                    for veh_id in obs_dicts[AgentType.VEHICLE]
                    if veh_id in existing_vehicle_list
                }
                if obs_dicts
                else {}
            ),
            AgentType.VULNERABLE_ROAD_USER: (
                {
                    vru_id: obs_dicts[AgentType.VULNERABLE_ROAD_USER][vru_id]
                    for vru_id in obs_dicts[AgentType.VULNERABLE_ROAD_USER]
                    if vru_id in existing_vru_list
                }
                if obs_dicts
                else {}
            ),
        }
        ctx_dicts = {
            AgentType.VEHICLE: (
                {
                    veh_id: ctx_dicts[AgentType.VEHICLE][veh_id]
                    for veh_id in ctx_dicts[AgentType.VEHICLE]
                    if veh_id in existing_vehicle_list
                }
                if ctx_dicts
                else {}
            ),
            AgentType.VULNERABLE_ROAD_USER: (
                {
                    vru_id: ctx_dicts[AgentType.VULNERABLE_ROAD_USER][vru_id]
                    for vru_id in ctx_dicts[AgentType.VULNERABLE_ROAD_USER]
                    if vru_id in existing_vru_list
                }
                if ctx_dicts
                else {}
            ),
        }
        return control_cmds, ctx_dicts, obs_dicts, self.should_continue_simulation()

    def cache_history_tls_data(self, focus_tls_ids=None):
        if not focus_tls_ids:
            focus_tls_ids = traci.trafficlight.getIDList()
        for tls_id in focus_tls_ids:
            if tls_id not in self.tls_info_cache:
                self.tls_info_cache[tls_id] = deque(maxlen=self.history_length + 1)
            current_time = traci.simulation.getTime()
            tls_state = traci.trafficlight.getRedYellowGreenState(tls_id)
            self.tls_info_cache[tls_id].append((current_time, tls_state))

    def sumo_warmup(self, warmup_time):
        # TODO: change vehicle type during the warmup time (might make warmup time longer)
        while True:
            while True:
                traci.simulationStep()
                if traci.simulation.getTime() > warmup_time:
                    break
            if traci.vehicle.getIDCount() > 500:
                logger.warning(
                    f"Too many vehicles in the simulation: {traci.vehicle.getIDCount()}, Restarting..."
                )
                traci.load(self.simulator.sumo_cmd[1:])
            else:
                break
        self.record.warmup_vehicle_num = traci.vehicle.getIDCount()
        self._vehicle_in_env_distance("before")

    def on_step(self, ctx):
        control_cmds, infos = self.make_decisions(ctx)
        self.refresh_control_commands_state()
        self.execute_control_commands(control_cmds)
        return self.should_continue_simulation()

    def refresh_control_commands_state(self):
        current_time = traci.simulation.getTime()
        for veh_id in self.vehicle_list.keys():
            self.vehicle_list[veh_id].controller._update_controller_status(
                veh_id, current_time
            )
        for vru_id in self.vulnerable_road_user_list.keys():
            self.vulnerable_road_user_list[vru_id].controller._update_controller_status(
                vru_id, current_time
            )

    def _vehicle_in_env_distance(self, mode):
        veh_id_list = traci.vehicle.getIDList()
        distance_dist = self._get_distance(veh_id_list)

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
            self.record.update(
                {
                    "veh_1_id": veh_1_id,
                    "veh_1_obs": self.vehicle_list[veh_1_id].observation,
                    "veh_2_id": veh_2_id,
                    "veh_2_obs": self.vehicle_list[veh_2_id].observation,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                    "finish_reason": "collision",
                }
            )
            return False
        elif utils.get_time() >= self.warmup_time + self.run_time:
            self.record.update(
                {
                    "veh_1_id": None,
                    "veh_2_id": None,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                    "finish_reason": "timeout",
                }
            )
            return False
        return True
