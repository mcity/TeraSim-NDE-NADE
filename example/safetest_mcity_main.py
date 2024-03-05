import os, sys
dir_path = os.path.dirname(os.path.realpath(__file__))

from terasim.simulator import Simulator
from terasim_nde_nade.envs.env_monitor import EnvMonitor
from terasim_nde_nade.envs.safetest_nade import SafeTestNADE
from terasim.logger.infoextractor import InfoExtractor
from terasim_nde_nade.vehicle.nde_vehicle_factory import NDEVehicleFactory
import argparse

parser = argparse.ArgumentParser(description='Run simulation.')
parser.add_argument('--dir', type=str, help='output directory', default="output")
parser.add_argument('--name', type=str, help='experiment name', default="test")
parser.add_argument('--nth', type=str, help='the nth epoch', default="0_0")
args = parser.parse_args()

monitor = EnvMonitor(
    log_dir=f"{args.dir}/{args.name}/raw_data",
    exp_id=f"{args.nth}",
)
env = SafeTestNADE(
    vehicle_factory = NDEVehicleFactory(lane_config_path=dir_path+'/maps/Mcity_safetest/lane_config.json'),
    info_extractor=InfoExtractor,
    warmup_time_lb=100,
    warmup_time_ub=200,
    run_time=1200,
)
sim = Simulator(
    sumo_net_file_path = dir_path+'/maps/Mcity_safetest/mcity.net.xml',
    sumo_config_file_path = dir_path+'/maps/Mcity_safetest/mcity.sumocfg',
    num_tries=10,
    gui_flag=False,
    output_path=f"{args.dir}/{args.name}/raw_data/{args.nth}",
    sumo_output_file_types=["fcd_all", "collision", "tripinfo"],
)
monitor.bind_env(env)
sim.bind_env(env)
sim.run()
