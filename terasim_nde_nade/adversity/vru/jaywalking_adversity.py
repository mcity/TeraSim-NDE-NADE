import addict
import math
import numpy as np

from terasim.overlay import traci
from terasim_nde_nade.adversity.abstract_adversity import AbstractAdversity
from terasim_nde_nade.utils import (
    CommandType,
    NDECommand,
    is_car_following,
)


def is_pedestrian_moving_forward(p_id, sumonet):
    edge_id = traci.person.getRoadID(p_id)
    edge_shape = sumonet.getEdge(edge_id).getShape()
    edge_start = edge_shape[0]
    edge_end = edge_shape[-1]
    edge_angle = (
        -math.degrees(
            math.atan(
                (edge_end[0] - edge_start[0]) / (edge_end[1] - edge_start[1])
            )
        )
        + 90
    ) % 360
    person_angle = traci.person.getAngle(p_id)
    if (edge_angle - person_angle) % 360 < 180:
        return True
    else:
        return False


class JaywalkingAdversity(AbstractAdversity):
    def __init__(self, location, ego_type, probability, predicted_collision_type):
        super().__init__(location, ego_type, probability, predicted_collision_type)
        self.sumo_net = None

    def trigger(self, obs_dict):
        self._negligence_command_dict = addict.Dict()

        ego_id = obs_dict["ego"]["vru_id"]
        edge_id = traci.person.getRoadID(ego_id)
        lane_id = traci.person.getLaneID(ego_id)
        lane_position = traci.person.getLanePosition(ego_id)
        lane_length = traci.lane.getLength(lane_id)
        if lane_position < 3 or lane_position > lane_length - 3:
            return False

        num_lanes: int = traci.edge.getLaneNumber(edge_id)
        has_sidewalk: bool = False
        has_road: bool = False
        have_separation: bool = False

        for i in range(num_lanes):
            lane_id = f"{edge_id}_{i}"
            allowed: list[str] = traci.lane.getAllowed(lane_id)
            disallowed: list[str] = traci.lane.getDisallowed(lane_id)
            if i == 0 and "pedestrian" in allowed:
                has_sidewalk = True
            if (
                "passenger" in allowed
                or "private" in allowed
                or "pedestrian" in disallowed
            ):
                has_road = True
            if i > 0 and i < num_lanes - 1 and "all" in disallowed:
                return False
        
        flag_is_moving_forward = is_pedestrian_moving_forward(ego_id, self.sumo_net)

        return has_sidewalk and has_road and (not have_separation) and flag_is_moving_forward
    
    def derive_command(self, obs_dict) -> addict.Dict:
        if self.trigger(obs_dict) and self._probability > 0:
            ego_id = obs_dict["ego"]["vru_id"]
            current_angle = traci.person.getAngle(ego_id)
            future_angle = (current_angle - 90) % 360  # 左转
            edge_id = traci.person.getRoadID(ego_id)
            total_width: float = sum([lane.getWidth() for lane in self.sumo_net.getEdge(edge_id).getLanes()])
            current_speed = traci.person.getSpeed(ego_id)
            if current_speed < 1.0: # if stop now, then the vru will accelerate to the max speed to cross the road
                speed = traci.person.getMaxSpeed(ego_id)
            else:
                speed = current_speed
                
            duration = round(total_width / speed, 1)

            trajectory = []
            current_pos = traci.person.getPosition(ego_id)
            distance = speed * (traci.simulation.getDeltaT())
            radians = math.radians(future_angle)
            new_pos = (
                current_pos[0] + distance * math.sin(radians),
                current_pos[1] + distance * math.cos(radians),
            )
            current_time = traci.simulation.getTime()
            dt = traci.simulation.getDeltaT()
            for i in range(int(duration / dt)):
                trajectory.append(
                    [
                        new_pos[0] + i * distance * math.sin(radians), # x
                        new_pos[1] + i * distance * math.cos(radians), # y
                        future_angle, # angle
                        speed, # speed
                        current_time + i * dt, # time
                    ]
                )
                
            negligence_command = NDECommand(
                command_type=CommandType.TRAJECTORY,
                future_trajectory=trajectory,
                duration=duration,
                prob=self._probability,
                keep_route_mode=6,
            )
            negligence_command.info.update(
                {
                    "speed": speed,
                    "angle": future_angle,
                    "mode": "negligence",
                    "negligence_mode": "Jaywalking",
                    "time_resolution": 0.1,
                    "predicted_collision_type": self._predicted_collision_type,
                    "location": self._location,
                }
            )
            self._negligence_command_dict.update(addict.Dict({"Jaywalking": negligence_command}))
            return self._negligence_command_dict
        return addict.Dict()