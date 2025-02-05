from terasim_nde_nade.vehicle.nde_decision_model import NDEDecisionModel
from terasim.overlay import traci
from terasim_nde_nade.vehicle.nde_vehicle_utils import (
    Command,
    NDECommand,
    get_location,
)
from terasim_nde_nade.adversity.adversity_manager import AdversityManager

BaseModel = NDEDecisionModel


class ConflictGenerationModel(BaseModel):
    def __init__(self, cfg=None, reroute=True, dynamically_change_vtype=True, *args, **kwargs):
        super().__init__(reroute, dynamically_change_vtype, *args, **kwargs)
        self.adversity_manager = AdversityManager(cfg.adversity_cfg.vehicle)

    def derive_control_command_from_observation(self, obs_dict):
        safe_nde_control_command, _ = super().derive_control_command_from_observation(
            obs_dict
        )

        # change the IDM and MOBIL parameters based on the location
        vehicle_location = get_location(
            obs_dict["ego"]["veh_id"], obs_dict["ego"]["lane_id"]
        )

        if (
            "EG_1_3_1.136" in obs_dict["ego"]["lane_id"]
            and obs_dict["ego"]["position"][1] > 305
        ):
            return safe_nde_control_command, {
                "ndd_command_distribution": {
                    "normal": NDECommand(
                        command_type=Command.DEFAULT,
                        prob=1,
                        info={"vehicle_location": vehicle_location},
                    )
                }
            }

        command, command_dict = self.adversity_manager.derive_command(obs_dict)        

        return command, {"ndd_command_distribution": command_dict}