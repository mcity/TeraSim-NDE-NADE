from .nde_decision_model import NDEDecisionModel

from ..utils import AdversityManager, CommandType, NDECommand, get_location


BaseModel = NDEDecisionModel


class ConflictGenerationModel(BaseModel):
    def __init__(
        self, cfg=None, reroute=True, dynamically_change_vtype=True, *args, **kwargs
    ):
        """Initialize the ConflictGenerationModel module.

        Args:
            cfg (dict): Configuration of the model.
            reroute (bool): Flag to indicate if the vehicle should be rerouted.
            dynamically_change_vtype (bool): Flag to indicate if the vehicle type should be changed dynamically.
        """
        super().__init__(reroute, dynamically_change_vtype, *args, **kwargs)
        self.adversity_manager = AdversityManager(cfg.adversity_cfg.vehicle)

    def derive_control_command_from_observation(self, obs_dict):
        """Derive the control command based on the observation.

        Args:
            obs_dict (dict): Observation of the ego agent.

        Returns:
            CommandType: Control command to be executed by the ego agent.
            dict: Dictionary containing the normal and adversarial maneuvers.
        """
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
                        command_type=CommandType.DEFAULT,
                        prob=1,
                        info={"vehicle_location": vehicle_location},
                    )
                }
            }

        command, command_dict = self.adversity_manager.derive_command(obs_dict)

        return command, {"ndd_command_distribution": command_dict}
