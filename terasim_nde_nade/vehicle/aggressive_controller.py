from terasim.overlay import traci

from terasim_nde_nade.vehicle.nde_controller import NDEController


class AggressiveController(NDEController):
    def execute_control_command(self, veh_id, control_command, obs_dict):
        # set all checks off
        self.all_checks_off(veh_id)
