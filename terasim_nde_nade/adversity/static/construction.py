from loguru import logger

from terasim.overlay import traci

from ...utils import AbstractStaticAdversity


class ConstructionAdversity(AbstractStaticAdversity):
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
    
    def initialize(self):
        """Initialize the adversarial event.
        """
        assert self.is_effective(), "Adversarial event is not effective."
        traci.lane.setDisallowed(self._lane_id, ["all"])
