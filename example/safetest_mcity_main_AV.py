import sys
import os

dir_path = os.path.dirname(os.path.abspath(__file__))
from terasim.simulator import Simulator
from terasim_nde_nade.envs.safetest_nade_with_av import SafeTestNADEWithAV
from terasim.logger.infoextractor import InfoExtractor
from terasim_nde_nade.vehicle.nde_vehicle_factory import NDEVehicleFactory
from pathlib import Path
import argparse

parser = argparse.ArgumentParser(description="Run simulation.")
parser.add_argument("--dir", type=str, help="output directory", default="output_cav")
parser.add_argument("--name", type=str, help="experiment name", default="test")
parser.add_argument("--nth", type=str, help="the nth epoch", default="0_0")
args = parser.parse_args()
base_dir = Path(args.dir) / args.name / "raw_data" / args.nth

env = SafeTestNADEWithAV(
    vehicle_factory=NDEVehicleFactory(),
    info_extractor=InfoExtractor,
    log_flag=True,
    log_dir=base_dir,
    warmup_time_lb=100,
    warmup_time_ub=200,
    run_time=1200,
)
sim = Simulator(
    sumo_net_file_path=dir_path + "/maps/Mcity_safetest/mcity.net.xml",
    sumo_config_file_path=dir_path + "/maps/Mcity_safetest/mcity.sumocfg",
    num_tries=10,
    gui_flag=True,
    output_path=base_dir,
    sumo_output_file_types=["fcd_all", "collision", "tripinfo"],
)
sim.bind_env(env)
sim.run()
