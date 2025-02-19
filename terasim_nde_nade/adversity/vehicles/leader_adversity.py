import addict

from terasim_nde_nade.adversity.abstract_adversity import AbstractAdversity
from terasim_nde_nade.utils import (
    derive_leader_adversarial_command,
    get_location,
)


class LeaderAdversity(AbstractAdversity):
    def trigger(self, obs_dict) -> bool:
        """Determine when to trigger the LeaderAdversity module.

        Args:
            obs_dict (dict): Observation of the ego agent.

        Returns:
            bool: Flag to indicate if the LeaderAdversity module should be triggered.
        """
        self._adversarial_command_dict = addict.Dict()
        vehicle_location = get_location(
            obs_dict["ego"]["veh_id"], obs_dict["ego"]["lane_id"]
        )
        if vehicle_location == self._location:
            adversarial_command_dict = derive_leader_adversarial_command(obs_dict)
            for key, command in adversarial_command_dict.items():
                rear_end = command.info.get("is_car_following_flag", False)
                if rear_end:
                    self._adversarial_command_dict[key] = command
        if self._adversarial_command_dict:
            return True
        else:
            return False

    def derive_command(self, obs_dict) -> addict.Dict:
        """Derive the adversarial command based on the observation.

        Args:
            obs_dict (dict): Observation of the ego agent.

        Returns:
            addict.Dict: Adversarial command.
        """
        if self.trigger(obs_dict) and self._probability > 0:
            for key, command in self._adversarial_command_dict.items():
                command.prob = self._probability
                command.info.update(
                    {
                        "predicted_collision_type": self._predicted_collision_type,
                        "vehicle_location": self._location,
                    }
                )
            return self._adversarial_command_dict
        return addict.Dict()
