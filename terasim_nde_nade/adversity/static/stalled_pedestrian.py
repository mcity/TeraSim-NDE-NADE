from loguru import logger

from terasim.overlay import traci

from ...utils import AbstractStaticAdversity


def create_pedestrian_type(pedestrian_type="DEFAULT"):
    """Create a custom pedestrian type for different pedestrian categories.

    Args:
        pedestrian_type (str): The type of pedestrian.
        Available types: "DEFAULT", "CHILD", "ADULT", "ELDERLY", "DISABLED"

    Returns:
        str: The ID of the custom pedestrian type.
    """
    custom_type_id = f"PEDESTRIAN_{pedestrian_type}"
    
    if custom_type_id not in traci.persontype.getIDList():
        # Create pedestrian type based on default
        traci.persontype.copy("DEFAULT_PEDTYPE", custom_type_id)
        
        # Set pedestrian-specific parameters based on type
        if pedestrian_type == "CHILD":
            traci.persontype.setWidth(custom_type_id, 0.4)
            traci.persontype.setLength(custom_type_id, 0.4)
            traci.persontype.setHeight(custom_type_id, 1.2)
            traci.persontype.setMaxSpeed(custom_type_id, 1.0)  # Child walks slower
            traci.persontype.setColor(custom_type_id, (255, 200, 200, 255))  # Light pink
        elif pedestrian_type == "ADULT":
            traci.persontype.setWidth(custom_type_id, 0.6)
            traci.persontype.setLength(custom_type_id, 0.6)
            traci.persontype.setHeight(custom_type_id, 1.8)
            traci.persontype.setMaxSpeed(custom_type_id, 1.4)  # Normal walking speed
            traci.persontype.setColor(custom_type_id, (100, 100, 255, 255))  # Blue
        elif pedestrian_type == "ELDERLY":
            traci.persontype.setWidth(custom_type_id, 0.5)
            traci.persontype.setLength(custom_type_id, 0.5)
            traci.persontype.setHeight(custom_type_id, 1.7)
            traci.persontype.setMaxSpeed(custom_type_id, 0.8)  # Elderly walks slower
            traci.persontype.setColor(custom_type_id, (150, 150, 150, 255))  # Gray
        elif pedestrian_type == "DISABLED":
            traci.persontype.setWidth(custom_type_id, 0.8)  # Wheelchair width
            traci.persontype.setLength(custom_type_id, 1.2)  # Wheelchair length
            traci.persontype.setHeight(custom_type_id, 1.4)  # Seated height
            traci.persontype.setMaxSpeed(custom_type_id, 0.6)  # Wheelchair speed
            traci.persontype.setColor(custom_type_id, (255, 100, 100, 255))  # Red
        else:  # DEFAULT
            traci.persontype.setWidth(custom_type_id, 0.5)
            traci.persontype.setLength(custom_type_id, 0.5)
            traci.persontype.setHeight(custom_type_id, 1.75)
            traci.persontype.setMaxSpeed(custom_type_id, 1.2)
            traci.persontype.setColor(custom_type_id, (0, 255, 0, 255))  # Green
    
    return custom_type_id


class StalledPedestrianAdversity(AbstractStaticAdversity):
    """
    A static adversity that places stationary pedestrians in the simulation.
    
    This class can be used to simulate various scenarios where pedestrians are
    standing still, such as:
    - People waiting at bus stops
    - Street performers or vendors
    - Injured pedestrians
    - People having conversations
    - Pedestrians looking at their phones
    """

    def is_effective(self):
        """Check if the adversarial event is effective.

        Returns:
            bool: Flag to indicate if the adversarial event is effective.
        """
        
        # Check if lane/edge ID is provided
        if self._lane_id == "":
            logger.warning("Lane/Edge ID is not provided.")
            return False
        
        # Check if lane position is provided
        if self._lane_position == -1:
            logger.warning("Lane position is not provided.")
            return False
        
        # Validate edge exists and get its length
        try:
            # Check if it's a lane ID (contains '_') or edge ID
            if '_' in self._lane_id:
                # It's a lane ID
                edge_id = traci.lane.getEdgeID(self._lane_id)
                lane_length = traci.lane.getLength(self._lane_id)
            else:
                # It's an edge ID
                edge_id = self._lane_id
                lane_length = traci.edge.getLength(edge_id)
        except Exception as e:
            logger.warning(f"Failed to get length of the lane/edge {self._lane_id}: {e}")
            return False
        
        # Check if position is within valid range
        if self._lane_position > lane_length:
            logger.warning(f"Position {self._lane_position} is greater than the lane/edge length {lane_length}.")
            return False
        
        # Set default pedestrian type if not provided
        if self._object_type == "":
            logger.warning("Pedestrian type is not provided. Using default value 'DEFAULT'.")
            self._object_type = "DEFAULT"
        elif self._object_type.upper() in ["CHILD", "ADULT", "ELDERLY", "DISABLED"]:
            # Create custom pedestrian type
            self._object_type = create_pedestrian_type(self._object_type.upper())
        else:
            # Check if the pedestrian type exists
            try:
                pedestrian_type_list = traci.persontype.getIDList()
                if self._object_type not in pedestrian_type_list:
                    logger.warning(f"Pedestrian type {self._object_type} is not available. Using default value 'DEFAULT'.")
                    self._object_type = create_pedestrian_type("DEFAULT")
            except Exception as e:
                logger.warning(f"Error checking pedestrian types: {e}. Using default value 'DEFAULT'.")
                self._object_type = create_pedestrian_type("DEFAULT")
        
        return True
    
    def set_pedestrian_feature(self, pedestrian_id: str):
        """Set pedestrian-specific features to make it stationary.
        
        Args:
            pedestrian_id (str): ID of the pedestrian to configure
        """
        try:
            # Set pedestrian speed to 0 to make it stationary
            traci.person.setSpeed(pedestrian_id, 0)
            # Set pedestrian type
            traci.person.setType(pedestrian_id, self._object_type)
        except Exception as e:
            logger.warning(f"Failed to set pedestrian features for {pedestrian_id}: {e}")

    def add_pedestrian(self, pedestrian_id: str):
        """Add a pedestrian to the simulation.
        
        Args:
            pedestrian_id (str): ID of the pedestrian to add
        """
        try:
            # Determine if we're working with a lane or edge
            if '_' in self._lane_id:
                # It's a lane ID, get the edge
                edge_id = traci.lane.getEdgeID(self._lane_id)
            else:
                # It's an edge ID
                edge_id = self._lane_id
            
            # Add pedestrian to the simulation
            traci.person.add(
                pedestrian_id,
                edge=edge_id,
                pos=self._lane_position,
                depart=0,
                typeID=self._object_type
            )
            
            # Set pedestrian features
            self.set_pedestrian_feature(pedestrian_id)
            
            # Move pedestrian to exact position if it's a lane
            if '_' in self._lane_id:
                traci.person.moveTo(pedestrian_id, self._lane_id, self._lane_position)
            
            logger.info(f"Added stationary pedestrian {pedestrian_id} at {self._lane_id}:{self._lane_position}")
            
        except Exception as e:
            logger.error(f"Failed to add pedestrian {pedestrian_id}: {e}")
            raise
    
    def initialize(self, time: float):
        """Initialize the adversarial event.
        
        Args:
            time (float): Current simulation time
        """
        assert self.is_effective(), "Adversarial event is not effective."
        
        # Create unique pedestrian ID
        pedestrian_id = f"PED_{self._object_type}_stalled_pedestrian_{int(time)}"
        self._static_adversarial_object_id_list.append(pedestrian_id)
        
        # Get edge information
        if '_' in self._lane_id:
            edge_id = traci.lane.getEdgeID(self._lane_id)
            lane_index = self._lane_id.split("_")[-1]
        else:
            edge_id = self._lane_id
            lane_index = "0"  # Default lane index for edge
        
        # Add pedestrian to simulation
        self.add_pedestrian(pedestrian_id)
        
        # Set instance variables
        self._duration = 0
        self._is_active = True
        self.pedestrian_id = pedestrian_id
        self.edge_id = edge_id
        self.lane_index = lane_index
        self.lane_position = self._lane_position
        
        logger.info(f"Initialized stalled pedestrian adversity: {pedestrian_id} at {self._lane_id}:{self._lane_position}")

    def update(self, time: float):
        """Update the adversarial event.
        
        Args:
            time (float): Current simulation time
        """
        # Check if it's time to remove the pedestrian
        if self._is_active and self.end_time != -1 and time >= self.end_time:
            try:
                traci.person.remove(self.pedestrian_id)
                logger.info(f"Removed pedestrian {self.pedestrian_id} at time {time}")
            except Exception as e:
                logger.warning(f"Failed to remove pedestrian {self.pedestrian_id}: {e}")
            self._is_active = False
        
        # Maintain pedestrian position and state if still active
        if self._is_active:
            try:
                # Ensure pedestrian remains stationary
                traci.person.setSpeed(self.pedestrian_id, 0)
                
                # Maintain position if using lane ID
                if '_' in self._lane_id:
                    traci.person.moveTo(self.pedestrian_id, self._lane_id, self._lane_position)
                
                # Update duration
                self._duration += 1
                
            except Exception as e:
                logger.warning(f"Failed to update pedestrian {self.pedestrian_id}: {e}")
                # If pedestrian no longer exists, mark as inactive
                self._is_active = False

    def get_pedestrian_info(self):
        """Get information about the stalled pedestrian.
        
        Returns:
            dict: Information about the pedestrian including position, type, and status
        """
        if not self._is_active:
            return {"status": "inactive"}
        
        try:
            return {
                "pedestrian_id": self.pedestrian_id,
                "edge_id": self.edge_id,
                "lane_id": self._lane_id,
                "position": self.lane_position,
                "type": self._object_type,
                "duration": self._duration,
                "status": "active"
            }
        except Exception as e:
            logger.warning(f"Failed to get pedestrian info: {e}")
            return {"status": "error", "message": str(e)}