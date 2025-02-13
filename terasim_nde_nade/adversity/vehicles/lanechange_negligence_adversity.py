import addict

from terasim_nde_nade.vehicle.nde_vehicle_utils import get_location
from terasim_nde_nade.adversity.abstract_adversity import AbstractAdversity
from terasim_nde_nade.adversity.adversity import derive_lane_change_negligence_command


class LanechangeNegligenceAdversity(AbstractAdversity):
    def trigger(self, obs_dict):
        self._negligence_command_dict = addict.Dict()
        vehicle_location = get_location(
            obs_dict["ego"]["veh_id"], obs_dict["ego"]["lane_id"]
        )
        if vehicle_location == self._location:
            negligence_command_dict = derive_lane_change_negligence_command(obs_dict)
            for key, command in negligence_command_dict.items():
                negligence_mode = command.info.get("negligence_mode", None)
                if negligence_mode == "LeftFoll" or negligence_mode == "RightFoll":
                    self._negligence_command_dict[key] = command
        if self._negligence_command_dict:      
            return True
        else:
            return False
    
    def derive_command(self, obs_dict) -> addict.Dict:
        if self.trigger(obs_dict) and self._probability > 0:
            for key, command in self._negligence_command_dict.items():
                command.prob = self._probability
                command.info.update(
                    {
                        "predicted_collision_type": self._predicted_collision_type,
                        "vehicle_location": self._location,
                    }
                )
            return self._negligence_command_dict
        return addict.Dict()