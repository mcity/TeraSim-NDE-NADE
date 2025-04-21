from loguru import logger

from terasim.overlay import traci

from ...utils import AbstractStaticAdversity

from .stalled_object import StalledObjectAdversity


class DynamicObjectAdversity(StalledObjectAdversity):

    def set_vehicle_feature(self, vehicle_id: str):
        # traci.vehicle.setSpeedMode(vehicle_id, 39) # [0 1 0 0 1 1 1] safety speed/acc/decce check on[1, 1, 1], road prioroty check off[0], brake to avoid red light running check off[0], disregard traffic rules in the junction[1], speed limit check on[0]
        traci.vehicle.setSpeedMode(vehicle_id, 0)
        traci.vehicle.setLaneChangeMode(vehicle_id, 0)
        traci.vehicle.setSpeedFactor(vehicle_id, 1.5)
        traci.vehicletype.setParameter(self._object_type, "lcStrategic", "100.0")
        traci.vehicletype.setParameter(self._object_type, "lcCooperative", "0.0")
        traci.vehicletype.setParameter(self._object_type, "lcSpeedGain", "100.0")
        traci.vehicletype.setParameter(self._object_type, "lcKeepRight", "0.0")



    def initialize(self, time: float):
        super().initialize(time)
        dynamic_route = ["EG_35_1_14", "EG_1_3_1", "EG_1_3_1.61", "EG_1_3_1.136"]
        traci.route.add("dynamic_route", dynamic_route)
        if self.stalled_object_id not in traci.vehicle.getIDList():
            traci.vehicle.add(self.stalled_object_id, routeID="dynamic_route", typeID=self._object_type)
        else:
            traci.vehicle.setRouteID(self.stalled_object_id, "dynamic_route")
        traci.vehicle.setSpeed(self.stalled_object_id, 10) 


    def update(self, time: float):
        if self._is_active and self.end_time != -1 and time >= self.end_time:
            try:
                traci.vehicle.remove(self.stalled_object_id)
            except:
                logger.warning(f"Failed to remove the vehicle {self.stalled_object_id}.")
            self._is_active = False


        