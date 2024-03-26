from terasim.vehicle.sensors.ego import EgoSensor
from terasim.overlay import traci
from collections import deque
import numpy as np
from math import isclose
from scipy.interpolate import interp1d
from loguru import logger


class NDEEgoSensor(EgoSensor):
    DEFAULT_PARAMS = dict(
        fields={
            "velocity": traci.vehicle.getSpeed,
            "position": traci.vehicle.getPosition,
            "position3d": traci.vehicle.getPosition3D,
            "heading": traci.vehicle.getAngle,
            "edge_id": traci.vehicle.getRoadID,
            "lane_id": traci.vehicle.getLaneID,
            "lane_index": traci.vehicle.getLaneIndex,
            "acceleration": traci.vehicle.getAcceleration,
            "next_links": traci.vehicle.getNextLinks,
        }
    )

    def __init__(self, cache_history=False, cache_history_duration=1, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._deltaT = None
        self.cache_history = cache_history
        self.cache_history_duration = cache_history_duration
        self.cache_length = (
            int(cache_history_duration / self.deltaT) + 1
        )  # add one for the current observation
        logger.trace(
            f"Cache length is {self.cache_length}, deltaT is {self.deltaT}, cache_history_duration is {self.cache_history_duration}"
        )
        if self.cache_history:
            self._history = deque(maxlen=self.cache_length)

    def fetch(self) -> dict:
        fetch_data = self._fetch()
        if self.cache_history:
            current_time = traci.simulation.getTime()
            self._history.append((current_time, fetch_data))
        return fetch_data

    @property
    def deltaT(self):
        if self._deltaT is None:
            self._deltaT = traci.simulation.getDeltaT()
        return self._deltaT

    @property
    def history_array(self):
        obs = self.observation  # make sure the newest observation is updated
        history_array = self._get_history_array(self.cache_length)
        current_time = traci.simulation.getTime()
        starting_time = current_time - self.cache_history_duration
        if self.cache_history:
            if isclose(self._history[0][0], starting_time, rel_tol=1e-3) and isclose(
                self._history[-1][0], current_time, rel_tol=1e-3
            ):
                return history_array
            elif self._history[0][0] <= starting_time:
                interp_times = np.arange(starting_time, current_time, self.deltaT)
                times = history_array[:, 0]
                data = history_array[:, 1:]
                interp_func = interp1d(times, data, axis=0, fill_value="extrapolate")
                interp_data = interp_func(interp_times)
                return np.concatenate([interp_times[:, None], interp_data], axis=1)
            else:  # history is not long enough
                logger.debug("History is not long enough, return None by default.")
                return None
        else:
            raise ValueError(
                "History is not cached, please set cache_history=True and cache_number when intializing the sensor."
            )

    def _get_history_array(self, history_duration):
        history_list = []
        for time, data in self._history:
            history_list.append(
                [
                    time,
                    data["position"][0],
                    data["position"][1],
                    data["velocity"],
                    data["heading"],
                    data["acceleration"],
                    data["length"],
                    data["width"],
                    data["height"],
                ]
            )
        return np.array(history_list)

    def _fetch(self) -> dict:
        original_dict = super().fetch()
        next_links = original_dict["next_links"]
        original_dict["upcoming_lanes"] = [original_dict["lane_id"]]
        original_dict["upcoming_lanes"] += get_next_lane_id_set_from_next_links(
            next_links
        )  # include current lane
        original_dict["upcoming_foe_lane_id_set"] = get_upcoming_foe_lane_id_set(
            original_dict["upcoming_lanes"]
        )
        original_dict["length"] = self.length
        original_dict["width"] = self.width
        original_dict["height"] = self.height
        return original_dict


def get_upcoming_foe_lane_id_set(upcoming_lanes):
    veh1_foe_lane_id_set = set()
    for lane_id in upcoming_lanes:
        veh1_foe_lane_id_set = veh1_foe_lane_id_set.union(
            set(traci.lane.getInternalFoes(lane_id))
        )
    return veh1_foe_lane_id_set


def get_next_lane_id_set_from_next_links(next_links):
    if len(next_links) == 0:
        return []
    next_lane_id = next_links[0][0]
    via_lane_id = next_links[0][4]
    return [via_lane_id, next_lane_id]
