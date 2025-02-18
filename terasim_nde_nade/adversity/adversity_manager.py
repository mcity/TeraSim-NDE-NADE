import random

from addict import Dict

from terasim_nde_nade.utils import CommandType, NDECommand
from terasim_nde_nade.utils.adversity.adversity_builder import build_adversities


class AdversityManager:
    def __init__(self, config):
        self.config = config
        self.adversities = []
        if self.config is not None:
            self.adversities = build_adversities(self.config)

    def derive_command(self, obs_dict):
        # get adversarial command candidates
        adversarial_command_dict = Dict()
        for adversity in self.adversities:
            adversity_output = adversity.derive_command(obs_dict)
            if adversity_output:
                adversarial_command_dict.update(Dict(adversity_output))
                break

        # Create command_dict based on filtered adversarial commands
        command_dict = {}
        if adversarial_command_dict:
            adversarial_command = list(adversarial_command_dict.values())[0]
            command_dict["adversarial"] = adversarial_command
            normal_prob = 1 - adversarial_command.prob
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
