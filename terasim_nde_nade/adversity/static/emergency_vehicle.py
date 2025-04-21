from loguru import logger

from terasim.overlay import traci

from ...utils import AbstractStaticAdversity


class EmergencyVehicleAdversity(AbstractStaticAdversity):

    def __init__(
        self,
        lane_id,
        lane_position=-1,
        object_type="emergency",
        other_settings={
            "guiShape": "police", # police, emergency(ambulance), firebrigade
            "speedFactor": 1.5,
            "has.bluelight.device": True
        }
    ):
        """Initialize the AbstractStaticAdversity class. This class is an abstract class that defines the interface for the different types of adversities that can be triggered in the simulation.

        Args:
            lane_id (str): Lane ID of the adversarial event.
            lane_position (int): Lane position of the adversarial event. Default is -1.
            object_type (str): Type of the object. Default is an empty string.
            other_settings (dict): Other settings for the adversarial event. Default is None.
        """
        self._lane_id = lane_id
        self._lane_position = lane_position
        self._object_type = object_type
        self._static_adversarial_object_id_list = []
        self._other_settings = other_settings


    def is_effective(self):
        """Check if the adversarial event is effective.

        Returns:
            bool: Flag to indicate if the adversarial event is effective.
        """
        if self._lane_id == "":
            logger.warning("Lane ID is not provided.")
            return False
        try:
            allowed_type_list = traci.lane.getAllowed(self._lane_id)
        except:
            logger.warning(f"Failed to get allowed types for lane {self._lane_id}.")
            return False
        return True
    
    def initialize(self, time: float):
        """Initialize the adversarial event.
        """
        assert self.is_effective(), "Adversarial event is not effective."
        # Create emergency vehicle type with specified settings
        emergency_type_id = f"emergency_{self._object_type}"
        if emergency_type_id not in traci.vehicletype.getIDList():
            traci.vehicletype.copy("DEFAULT_VEHTYPE", emergency_type_id)
            traci.vehicletype.setVClass(emergency_type_id, self._object_type)
            
            # Apply other settings
            if "guiShape" in self._other_settings:
                traci.vehicletype.setShapeClass(emergency_type_id, self._other_settings["guiShape"])
            if "speedFactor" in self._other_settings:
                traci.vehicletype.setSpeedFactor(emergency_type_id, self._other_settings["speedFactor"])
            if "has.bluelight.device" in self._other_settings:
                traci.vehicletype.setParameter(emergency_type_id, "has.bluelight.device", "true")

        # Create emergency vehicle route
        emergency_route_id = f"emergency_route_{self._object_type}"
        traci.route.add(emergency_route_id, [self._lane_id])

        # Create emergency vehicle
        emergency_vehicle_id = f"emergency_{self._object_type}"
        traci.vehicle.add(emergency_vehicle_id, emergency_route_id, typeID=emergency_type_id)
        traci.vehicle.setSpeed(emergency_vehicle_id, 0)
        traci.vehicle.moveTo(emergency_vehicle_id, self._lane_id, self._lane_position)

        self._duration = 0
        self._is_active = True
        
        