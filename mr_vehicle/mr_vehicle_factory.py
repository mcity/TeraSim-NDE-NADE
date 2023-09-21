from mr_vehicle.mr_controller import MRController
from mr_vehicle.mr_decision_model import MRDecisionModelLocal, MRDecisionModelRemote
from mr_vehicle.mr_context_sensor import MRContextSensorLocal, MRContextSensorRemote
from mr_vehicle.mr_tls_sensor import MRTLSSensorLocal, MRTLSSensorRemote
from mtlsp.vehicle.factories.dummy_vehicle_factory import DummyVehicleFactory
from vehicle.IDM_MOBIL_with_negligence import IDM_MOBIL_with_negligence
import settings
from mtlsp.vehicle.sensors.ego import EgoSensor
from mtlsp.vehicle.sensors.local import LocalSensor
from mtlsp.vehicle.vehicle import Vehicle
from vehicle.high_efficiency_controller_v2 import HighEfficiencyControllerV2

class MRVehicleFactoryLocal(DummyVehicleFactory):
    def create_vehicle(self, veh_id, simulator):
        if "CAV" in veh_id:
            sensor_list = [EgoSensor(), LocalSensor(), MRContextSensorLocal(), MRTLSSensorLocal(sumo_tls_addition_info_file_path=settings.sumo_tls_addition_info_file_path)]
            decision_model = MRDecisionModelLocal()
            # decision_model = IDMModel(MOBIL_lc_flag=True, stochastic_acc_flag=True)
            control_params = {
                "v_high": 20,
                "v_low": 0,
                "acc_duration": 0.1,  # the acceleration duration will be 0.1 second
                "lc_duration": 1,  # the lane change duration will be 1 second
                "neg_duration": 2, # the negligence duration will be 2 second
            }
            controller = MRController(simulator, control_params)
        else:
            sensor_list = [EgoSensor(), LocalSensor()]
            decision_model = IDM_MOBIL_with_negligence(MOBIL_lc_flag=True, stochastic_acc_flag=True)
            # decision_model = IDMModel(MOBIL_lc_flag=True, stochastic_acc_flag=True)
            control_params = {
                "v_high": 20,
                "v_low": 0,
                "acc_duration": 0.1,  # the acceleration duration will be 0.1 second
                "lc_duration": 1,  # the lane change duration will be 1 second
                "neg_duration": 2, # the negligence duration will be 2 second
            }
            controller = HighEfficiencyControllerV2(simulator, control_params)
        return Vehicle(veh_id, simulator, sensors=sensor_list,
                       decision_model=decision_model, controller=controller)
    

class MRVehicleFactoryRemote(DummyVehicleFactory):
    def create_vehicle(self, veh_id, simulator):
        if "CAV" in veh_id:
            sensor_list = [EgoSensor(), LocalSensor(), MRContextSensorRemote(), MRTLSSensorRemote(sumo_tls_addition_info_file_path=settings.sumo_tls_addition_info_file_path)]
            decision_model = MRDecisionModelRemote()
            # decision_model = IDMModel(MOBIL_lc_flag=True, stochastic_acc_flag=True)
            control_params = {
                "v_high": 20,
                "v_low": 0,
                "acc_duration": 0.1,  # the acceleration duration will be 0.1 second
                "lc_duration": 1,  # the lane change duration will be 1 second
                "neg_duration": 2, # the negligence duration will be 2 second
            }
            controller = MRController(simulator, control_params)
        else:
            sensor_list = [EgoSensor(), LocalSensor()]
            decision_model = IDM_MOBIL_with_negligence(MOBIL_lc_flag=True, stochastic_acc_flag=True)
            # decision_model = IDMModel(MOBIL_lc_flag=True, stochastic_acc_flag=True)
            control_params = {
                "v_high": 20,
                "v_low": 0,
                "acc_duration": 0.1,  # the acceleration duration will be 0.1 second
                "lc_duration": 1,  # the lane change duration will be 1 second
                "neg_duration": 2, # the negligence duration will be 2 second
            }
            controller = HighEfficiencyControllerV2(simulator, control_params)
        return Vehicle(veh_id, simulator, sensors=sensor_list,
                       decision_model=decision_model, controller=controller)