from addict import Dict
import random

from terasim_nde_nade.utils.adversity.adversity_builder import build_adversities
from terasim_nde_nade.utils import (
    CommandType,
    NDECommand,
)

class AdversityManager():
    def __init__(self, config):
        self.config = config
        self.adversities = []
        if self.config is not None:
            self.adversities = build_adversities(self.config)
        
    def derive_command(self, obs_dict):
        # get negligence command candidates
        negligence_command_dict = Dict()
        for adversity in self.adversities:
            adversity_output = adversity.derive_command(obs_dict)
            if adversity_output:
                negligence_command_dict.update(Dict(adversity_output))
                break

        # Create command_dict based on filtered negligence commands
        command_dict = {}
        if negligence_command_dict:
            negligence_command = list(negligence_command_dict.values())[0]
            command_dict["negligence"] = negligence_command
            normal_prob = 1 - negligence_command.prob
        else:
            normal_prob = 1

        # vehicle_location = get_location(
        #     obs_dict["ego"]["veh_id"], obs_dict["ego"]["lane_id"]
        # )
        command_dict["normal"] = NDECommand(
            command_type=CommandType.DEFAULT,
            prob=normal_prob,
            # info={"vehicle_location": vehicle_location},
        )

        # Sample final command based on the probability in command_dict
        command = random.choices(
            list(command_dict.values()),
            weights=[cmd.prob for cmd in command_dict.values()],
            k=1,
        )[0]

        return command, command_dict