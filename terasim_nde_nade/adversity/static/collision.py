from loguru import logger
import random

from terasim.overlay import traci

from ...utils import AbstractStaticAdversity


class CollisionAdversity(AbstractStaticAdversity):
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
        if isinstance(self._object_type, str):
            vehicle_type_list = traci.vehicletype.getIDList()
            if self._object_type not in vehicle_type_list:
                logger.warning(f"Vehicle type {self._object_type} is not available. Using default value 'DEFAULT_VEHTYPE'.")
                self._object_type = ["DEFAULT_VEHTYPE"] * 2
            else:
                self._object_type = [self._object_type] * 2
        elif isinstance(self._object_type, list):
            vehicle_type_list = traci.vehicletype.getIDList()
            for index, object_type in enumerate(self._object_type):
                if object_type not in vehicle_type_list:
                    logger.warning(f"Vehicle type {object_type} is not available. Using default value 'DEFAULT_VEHTYPE'.")
                    self._object_type[index] = "DEFAULT_VEHTYPE"
        else:
            logger.warning("Object type is not provided. Using default value 'DEFAULT_VEHTYPE'.")
            self._object_type = ["DEFAULT_VEHTYPE"] * 2
        if self._other_settings is None:
            logger.warning("Details are not provided.")
            return False
        return True
    
    def execute(self):
        """Execute the adversarial event.
        """
        assert self.is_effective(), "Adversarial event is not effective."
        object1_id = f"BV_1_{self._object_type[0]}_collision"
        object2_id = f"BV_2_{self._object_type[1]}_collision"
        self._static_adversarial_object_id_list.append(object1_id)
        self._static_adversarial_object_id_list.append(object2_id)
        edge_id = traci.lane.getEdgeID(self._lane_id)
        collision_object_route_id = f"r_stalled_object"
        traci.route.add(collision_object_route_id, [edge_id])
        traci.vehicle.add(
            object1_id,
            routeID=collision_object_route_id,
            typeID=self._object_type[0],
        )

        traci.vehicle.moveTo(object1_id, self._lane_id, self._lane_position)
        traci.vehicle.setSpeed(object1_id, 0)
        traci.vehicle.setSpeedMode(object1_id, 0)
        traci.vehicle.setLaneChangeMode(object1_id, 0)
        traci.vehicle.add(
            object2_id,
            routeID=collision_object_route_id,
            typeID=self._object_type[1],
        )
        if "collision_type" in self._other_settings:
            collision_type = self._other_settings["collision_type"]
        else:
            collision_type = "rear_end"
        if collision_type == "rear_end":
            object1_length = traci.vehicletype.getLength(self._object_type[0])
            traci.vehicle.moveTo(object2_id, self._lane_id, self._lane_position - object1_length)
        elif collision_type == "side_swipe":
            lane_index = int(self._lane_id.split("_")[-1])
            if lane_index == 0:
                angle_diff = -20.0
            elif lane_index == traci.edge.getLaneNumber(edge_id) - 1:
                angle_diff = 20.0
            else:
                angle_diff = random.choice([-10.0, 10.0])
            object2_length = traci.vehicletype.getLength(self._object_type[1])
            lane_position_object2 = random.choice(
                [self._lane_position - object2_length / 2, self._lane_position + object2_length / 2]
            )
            traci.vehicle.moveTo(object2_id, self._lane_id, lane_position_object2)
            object2_position = traci.vehicle.getPosition(object2_id)
            object2_angle = traci.vehicle.getAngle(object2_id)
            traci.vehicle.moveToXY(
                object2_id, 
                edge_id, 
                lane_index, 
                object2_position[0],
                object2_position[1],
                object2_angle + angle_diff
            )
        else:
            raise ValueError(f"Collision type {collision_type} is not supported.")
        traci.vehicle.setSpeed(object2_id, 0)
        traci.vehicle.setSpeedMode(object2_id, 0)
        traci.vehicle.setLaneChangeMode(object2_id, 0)