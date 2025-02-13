from terasim.vulnerable_road_user.factories.vulnerable_road_user_factory import (
    VulnerableRoadUserFactory,
)
from terasim.vulnerable_road_user.sensors.ego import EgoSensor
from terasim.vulnerable_road_user.vulnerable_road_user import VulnerableRoadUser

from terasim_nde_nade.vru.conflict_generation_model import ConflictGenerationModel
from terasim_nde_nade.vru.nde_controller import NDEVulnerableRoadUserController


class NDEVulnerableRoadUserFactory(VulnerableRoadUserFactory):
    def __init__(self, cfg=None):
        super().__init__()
        self.cfg = cfg

    def create_vulnerable_road_user(self, vru_id, simulator):
        sensor_list = [
            EgoSensor(cache=True, cache_history=True, cache_history_duration=1)
        ]
        decision_model = ConflictGenerationModel(
            cfg=self.cfg,
            MOBIL_lc_flag=self.cfg.MOBIL_lc_flag,
            stochastic_acc_flag=self.cfg.stochastic_acc_flag,
        )

        controller = NDEVulnerableRoadUserController(simulator)
        return VulnerableRoadUser(
            vru_id,
            simulator,
            sensors=sensor_list,
            decision_model=decision_model,
            controller=controller,
        )
