import redis
from terasim.overlay import traci
from loguru import logger
import numpy as np
import terasim.utils as utils
from terasim_nde_nade.envs.safetest_nade_with_av import SafeTestNADEWithAV


class SafeTestNADEWithAVCosim(SafeTestNADEWithAV):

    def on_start(self, ctx):
        super().on_start(ctx)
        self.redis_client = redis.Redis(host="localhost", port=6379, db=0)
        self.redis_client.set("terasim_status", 1)

    def get_IS_prob(
        self,
        veh_id,
        ndd_control_command_dicts,
        maneuver_challenge_dicts,
        veh_ctx_dicts,
    ):
        if not maneuver_challenge_dicts[veh_id].get("negligence"):
            raise ValueError("The vehicle is not in the negligence mode.")

        # IS_magnitude = 10000000

        # lower_bound = 0.0
        # upper_bound = 0.1

        # predicted_collision_type = ndd_control_command_dicts[veh_id].negligence.info["predicted_collision_type"]

        # return np.clip(
        #     ndd_control_command_dicts[veh_id]["negligence"].prob * IS_magnitude,
        #     lower_bound,
        #     upper_bound,
        # )

        return 0.05

        # return self.max_importance_sampling_prob

    def should_continue_simulation(self):
        location = traci.vehicle.getPosition3D("CAV")
        x, y = location[0], location[1]

        terminate_flag = self.redis_client.get("terminate_TeraSim")

        num_colliding_vehicles = self.simulator.get_colliding_vehicle_number()

        if "CAV" not in traci.vehicle.getIDList():
            logger.info("CAV left the simulation, stop the simulation.")
            self.record.update(
                {
                    "veh_1_id": None,
                    "veh_2_id": None,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                    "finish_reason": "CAV_left",
                }
            )
            return False

        elif utils.get_time() >= self.warmup_time + self.run_time:
            logger.info("Simulation timeout, stop the simulation.")
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

        if terminate_flag and int(terminate_flag) == 1:
            logger.info(
                "Received termination signal from Redis, stopping the simulation."
            )
            self.record.update(
                {
                    "veh_1_id": None,
                    "veh_2_id": None,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                    "finish_reason": "finished",
                }
            )
            return False

        # elif y > 310.0:
        #     logger.info("CAV reached the destination, stop the simulation.")
        #     self.record.update(
        #         {
        #             "veh_1_id": None,
        #             "veh_2_id": None,
        #             "warmup_time": self.warmup_time,
        #             "run_time": self.run_time,
        #             "finish_reason": "finished",
        #         }
        #     )
        #     return False

        return True
