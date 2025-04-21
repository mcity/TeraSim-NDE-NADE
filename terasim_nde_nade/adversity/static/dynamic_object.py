from loguru import logger

from terasim.overlay import traci

from ...utils import AbstractStaticAdversity

from .stalled_object import StalledObjectAdversity


class DynamicObjectAdversity(StalledObjectAdversity):

    def update(self, time: float):
        if self._is_active and self.end_time != -1 and time >= self.end_time:
            try:
                traci.vehicle.remove(self.stalled_object_id)
            except:
                logger.warning(f"Failed to remove the vehicle {self.stalled_object_id}.")
            self._is_active = False


        