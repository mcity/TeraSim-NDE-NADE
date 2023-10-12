import sys
import os
file_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(file_path + "/../../")

from terasim.simulator import Simulator
from envs.env_monitor import EnvMonitor
from envs.safetest_nde import SafeTestNDE
from envs.safetest_nade import SafeTestNADE
from terasim.logger.infoextractor import InfoExtractor
from vehicle.safetest_vehicle_factory import SafetestVehicleFactory

import argparse


parser = argparse.ArgumentParser(description='Run simulation.')
parser.add_argument('--dir', type=str, help='output directory', default="output")
parser.add_argument('--exp_name', type=str, help='experiment name', default="test")
parser.add_argument('--nth', type=str, help='the nth epoch', default="0_0")
args = parser.parse_args()

highlight_routes = set(["7", "10", "13", "14", "16"])
monitor = EnvMonitor(
    highlight_routes=highlight_routes,
    log_dir=f"{args.dir}/{args.exp_name}/raw_data",
    exp_id=f"{args.exp_name}_{args.nth}",
)
env = SafeTestNADE(
    vehicle_factory = SafetestVehicleFactory(),
    info_extractor=InfoExtractor,
)
sim = Simulator(
    sumo_net_file_path = file_path + '/maps/Mcity_safetest/mcity.net.xml',
    sumo_config_file_path = file_path + '/maps/Mcity_safetest/mcity.sumocfg',
    num_tries=10,
    gui_flag=False,
    output_path=f"{args.dir}/{args.exp_name}/raw_data/{args.exp_name}_{args.nth}",
    sumo_output_file_types=["fcd_all", "collision", "tripinfo"],
)
monitor.bind_env(env)
sim.bind_env(env)
sim.run()
