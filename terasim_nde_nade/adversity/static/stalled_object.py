from loguru import logger

from terasim.overlay import traci

from ...utils import AbstractStaticAdversity


class StalledObjectAdversity(AbstractStaticAdversity):
    def is_effective(self):
        """Check if the adversarial event is effective.

        Returns:
            bool: Flag to indicate if the adversarial event is effective.
        """
        if self._lane_id == "":
            logger.warning("Lane ID is not provided.")
            return False
        if self._lane_position == -1:
            logger.warning("Lane position is not provided.")
            return False
        try:
            lane_length = traci.lane.getLength(self._lane_id)
        except:
            logger.warning(f"Failed to get length of the lane {self._lane_id}.")
            return False
        if self._lane_position > lane_length:
            logger.warning(f"Lane position {self._lane_position} is greater than the lane length {lane_length}.")
            return False
        if self._object_type == "":
            logger.warning("Object type is not provided. Using default value 'DEFAULT_VEHTYPE'.")
            self._object_type = "DEFAULT_VEHTYPE"
        else:
            vehicle_type_list = traci.vehicletype.getIDList()
            if self._object_type not in vehicle_type_list:
                logger.warning(f"Vehicle type {self._object_type} is not available. Using default value 'DEFAULT_VEHTYPE'.")
                self._object_type = "DEFAULT_VEHTYPE"
        return True
    
    def initialize(self):
        """Initialize the adversarial event.
        """
        assert self.is_effective(), "Adversarial event is not effective."
        stalled_object_id = f"BV_{self._object_type}_stalled_object"
        self._static_adversarial_object_id_list.append(stalled_object_id)
        edge_id = traci.lane.getEdgeID(self._lane_id)
        stalled_object_route_id = f"r_stalled_object"
        traci.route.add(stalled_object_route_id, [edge_id])
        traci.vehicle.add(
            stalled_object_id,
            routeID=stalled_object_route_id,
            typeID=self._object_type,
        )
        traci.vehicle.setSpeedMode(stalled_object_id, 0)
        traci.vehicle.setLaneChangeMode(stalled_object_id, 0)        
        traci.vehicle.moveTo(stalled_object_id, self._lane_id, self._lane_position)
        traci.vehicle.setSpeed(stalled_object_id, 0)

        self._duration=0
        self._is_active = True
        self.stalled_object_id = stalled_object_id

    def update(self):
        if "duration" in self._other_settings:
            self._duration += traci.simulation.getDeltaT()
            if self._is_active and self._duration >= self._other_settings["duration"]:
                try:
                    traci.vehicle.remove(self.stalled_object_id)
                except:
                    logger.warning(f"Failed to remove the vehicle {self.stalled_object_id}.")
                self._is_active = False

    