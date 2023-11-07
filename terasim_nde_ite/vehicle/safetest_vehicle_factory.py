from terasim.vehicle.factories.dummy_vehicle_factory import DummyVehicleFactory
from terasim.vehicle.sensors.ego import EgoSensor
from terasim.vehicle.sensors.local import LocalSensor
from terasim.vehicle.vehicle import Vehicle
from terasim_nde_ite.vehicle.high_efficiency_controller_ite import HighEfficiencyControllerITE
from terasim_nde_ite.vehicle.IDM_MOBIL_with_negligence import IDM_MOBIL_with_negligence
import json

class SafetestVehicleFactory(DummyVehicleFactory):
    def __init__(self, lane_config_path) -> None:
        self.lane_config = json.load(open(lane_config_path, "r"))
        super().__init__()

    def create_vehicle(self, veh_id, simulator):
        sensor_list = [EgoSensor(), LocalSensor()]
        decision_model = IDM_MOBIL_with_negligence(MOBIL_lc_flag=True, stochastic_acc_flag=False, lane_config=self.lane_config)
        # decision_model = IDMModel(MOBIL_lc_flag=True, stochastic_acc_flag=True)
        control_params = {
            "v_high": 20,
            "v_low": 0,
            "acc_duration": 0.1,  # the acceleration duration will be 0.1 second
            "lc_duration": 1,  # the lane change duration will be 1 second
            "neg_duration": 2, # the negligence duration will be 2 second
        }
        controller = HighEfficiencyControllerITE(simulator, control_params)
        return Vehicle(veh_id, simulator, sensors=sensor_list,
                       decision_model=decision_model, controller=controller)


class SafetestDummmyVehicleFactory(DummyVehicleFactory):
    def create_vehicle(self, veh_id, simulator):
        sensor_list = []
        decision_model = IDM_MOBIL_with_negligence(MOBIL_lc_flag=True, stochastic_acc_flag=True)
        # decision_model = IDMModel(MOBIL_lc_flag=True, stochastic_acc_flag=True)
        control_params = {
            "v_high": 20,
            "v_low": 0,
            "acc_duration": 0.1,  # the acceleration duration will be 0.1 second
            "lc_duration": 1,  # the lane change duration will be 1 second
            "neg_duration": 2, # the negligence duration will be 2 second
        }
        controller = HighEfficiencyControllerITE(simulator, control_params)
        return Vehicle(veh_id, simulator, sensors=sensor_list,
                       decision_model=decision_model, controller=controller)