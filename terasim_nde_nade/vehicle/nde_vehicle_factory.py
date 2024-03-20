from terasim.vehicle.factories.vehicle_factory import VehicleFactory
from terasim.vehicle.sensors.ego import EgoSensor
from terasim.vehicle.sensors.local import LocalSensor
from terasim.vehicle.vehicle import Vehicle
from terasim_nde_nade.vehicle.nde_controller import NDEController
from terasim_nde_nade.vehicle.aggressive_controller import AggressiveController
from terasim_nde_nade.vehicle.nde_decision_model import NDEDecisionModel
from terasim_nde_nade.vehicle.nde_ego_sensor import NDEEgoSensor
import json


class NDEVehicleFactory(VehicleFactory):
    def __init__(self, edge_config_path) -> None:
        self.edge_config = json.load(open(edge_config_path, "r"))
        super().__init__()

    def create_vehicle(self, veh_id, simulator):

        sensor_list = [NDEEgoSensor()]
        decision_model = NDEDecisionModel(MOBIL_lc_flag=True, stochastic_acc_flag=False)
        # decision_model = IDMModel(MOBIL_lc_flag=True, stochastic_acc_flag=True)
        control_params = {
            "v_high": 20,
            "v_low": 0,
            "acc_duration": 0.1,  # the acceleration duration will be 0.1 second
            "lc_duration": 1,  # the lane change duration will be 1 second
            "neg_duration": 2,  # the negligence duration will be 2 second
        }
        # if veh_id == "CAV":
        if False:
            controller = AggressiveController(simulator, control_params)
        else:
            controller = NDEController(simulator, control_params)
        return Vehicle(
            veh_id,
            simulator,
            sensors=sensor_list,
            decision_model=decision_model,
            controller=controller,
        )
