import addict

from terasim_nde_nade.adversity.abstract_adversity import AbstractAdversity
from terasim_nde_nade.utils import get_location
from terasim_nde_nade.utils.adversity import derive_traffic_rule_adversarial_command


class TrafficRuleAdversity(AbstractAdversity):
    def trigger(self, obs_dict):
        self._adversarial_command_dict = addict.Dict()
        vehicle_location = get_location(
            obs_dict["ego"]["veh_id"], obs_dict["ego"]["lane_id"]
        )
        if vehicle_location == self._location:
            adversarial_command_dict = derive_traffic_rule_adversarial_command(obs_dict)
            for key, command in adversarial_command_dict.items():
                if command.info.get("adversarial_mode", None) == "TrafficRule":
                    self._adversarial_command_dict[key] = command
        if self._adversarial_command_dict:
            return True
        else:
            return False

    def derive_command(self, obs_dict) -> addict.Dict:
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
