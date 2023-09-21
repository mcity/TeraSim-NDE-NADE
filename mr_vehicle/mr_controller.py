from mtlsp.vehicle.controllers.base_controller import BaseController


class MRController(BaseController):

    def _is_command_legal(self, veh_id, control_command):
        if control_command is not None:
            return self.simulator.command_manager._is_command_legal(veh_id, control_command)
        else:
            return False

    def execute_control_command(self, veh_id, control_command, obs_dict):
        return self.simulator.command_manager.execute_control_command(veh_id, control_command, obs_dict)