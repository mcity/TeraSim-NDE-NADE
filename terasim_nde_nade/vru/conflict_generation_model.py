from terasim.overlay import traci

from terasim_nde_nade.adversity.adversity_manager import AdversityManager
from terasim_nde_nade.utils import CommandType, NDECommand, get_location
from terasim_nde_nade.vehicle.nde_decision_model import NDEDecisionModel

BaseModel = NDEDecisionModel


class ConflictGenerationModel(BaseModel):
    def __init__(
        self, cfg=None, reroute=True, dynamically_change_vtype=True, *args, **kwargs
    ):
        super().__init__(reroute, dynamically_change_vtype, *args, **kwargs)
        self.adversity_manager = AdversityManager(
            cfg.adversity_cfg.vulnerable_road_user
        )

    def derive_control_command_from_observation(self, obs_dict):
        for adversity in self.adversity_manager.adversities:
            if adversity.sumo_net is None:
                adversity.sumo_net = self._agent.simulator.sumo_net
        command, command_dict = self.adversity_manager.derive_command(obs_dict)

        # return None, None
        return command, {"ndd_command_distribution": command_dict}
