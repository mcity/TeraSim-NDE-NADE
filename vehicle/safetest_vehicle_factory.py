from mtlsp.vehicle.factories.dummy_vehicle_factory import DummyVehicleFactory
from mtlsp.vehicle.sensors.ego import EgoSensor
from mtlsp.vehicle.sensors.local import LocalSensor
from mtlsp.vehicle.controllers.high_efficiency_controller import HighEfficiencyController
from mtlsp.vehicle.vehicle import Vehicle
from vehicle.high_efficiency_controller_v2 import HighEfficiencyControllerV2
from vehicle.IDM_MOBIL_with_negligence import IDM_MOBIL_with_negligence
from mtlsp.vehicle.decision_models.idm_model import IDMModel

from ascs.vehicle.controllers.autowareai_controller import AutowareAIMoveController
from ascs.vehicle.decision_models.autowareai_decision_model import AutowareAIDecisionModel_ROS, AutowareAIDecisionModel_Web
from ascs.vehicle.sensors.autowareai_context_sensor import AutowareAIContextSensor_ROS, AutowareAIContextSensor_Web
from ascs.vehicle.sensors.autowareai_tls_sensor import AutowareAITLSSensor_ROS, AutowareAITLSSensor_Web
import settings


class SafetestVehicleFactory(DummyVehicleFactory):
    def create_vehicle(self, veh_id, simulator):
        sensor_list = [EgoSensor(), LocalSensor()]
        decision_model = IDM_MOBIL_with_negligence(MOBIL_lc_flag=True, stochastic_acc_flag=False)
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
        controller = HighEfficiencyControllerV2(simulator, control_params)
        return Vehicle(veh_id, simulator, sensors=sensor_list,
                       decision_model=decision_model, controller=controller)



class SafetestCosimVehicleFactory_Local(DummyVehicleFactory):
    def create_vehicle(self, veh_id, simulator):
        if "CAV" in veh_id:
            sensor_list = [EgoSensor(), LocalSensor(), AutowareAIContextSensor_ROS(), AutowareAITLSSensor_ROS(sumo_tls_addition_info_file_path=settings.sumo_tls_addition_info_file_path)]
            decision_model = AutowareAIDecisionModel_ROS()
            # decision_model = IDMModel(MOBIL_lc_flag=True, stochastic_acc_flag=True)
            control_params = {
                "v_high": 20,
                "v_low": 0,
                "acc_duration": 0.1,  # the acceleration duration will be 0.1 second
                "lc_duration": 1,  # the lane change duration will be 1 second
                "neg_duration": 2, # the negligence duration will be 2 second
            }
            controller = AutowareAIMoveController(simulator, control_params)
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
    

class SafetestCosimVehicleFactory_Remote(DummyVehicleFactory):
    def create_vehicle(self, veh_id, simulator):
        if "CAV" in veh_id:
            sensor_list = [EgoSensor(), LocalSensor(), AutowareAIContextSensor_Web(), AutowareAITLSSensor_Web(sumo_tls_addition_info_file_path=settings.sumo_tls_addition_info_file_path)]
            decision_model = AutowareAIDecisionModel_Web()
            # decision_model = IDMModel(MOBIL_lc_flag=True, stochastic_acc_flag=True)
            control_params = {
                "v_high": 20,
                "v_low": 0,
                "acc_duration": 0.1,  # the acceleration duration will be 0.1 second
                "lc_duration": 1,  # the lane change duration will be 1 second
                "neg_duration": 2, # the negligence duration will be 2 second
            }
            controller = AutowareAIMoveController(simulator, control_params)
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