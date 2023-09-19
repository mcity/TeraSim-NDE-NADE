from mtlsp.simulator import Simulator
from envs.env_monitor import EnvMonitor
from envs.safetest_nde import SafeTestNDE
from envs.safetest_nade import SafeTestNADE
from envs.safetest_nde_sumo import SafeTestNDESUMO
from mtlsp.logger.infoextractor import InfoExtractor
from vehicle.safetest_vehicle_factory import SafetestVehicleFactory
from vehicle.safetest_vehicle_factory import SafetestDummmyVehicleFactory

import argparse

parser = argparse.ArgumentParser(description='Run simulation.')
parser.add_argument('--dir', type=str, help='output directory', default="output")
parser.add_argument('--mode', type=str, help='the negligence mode.', default="test")
parser.add_argument('--nth', type=str, help='the nth epoch', default="0_0")
args = parser.parse_args()

highlight_routes = set(["7", "10", "13", "14", "16"])
monitor = EnvMonitor(
    highlight_routes=highlight_routes,
    filename=f"{args.dir}/{args.mode}/{args.mode}_{args.nth}/monitor.json",
)
env = SafeTestNDESUMO(
    vehicle_factory = SafetestDummmyVehicleFactory(),
    info_extractor=InfoExtractor,
)
sim = Simulator(
    sumo_net_file_path = './maps/Mcity/mcity.net.xml',
    sumo_config_file_path = './maps/Mcity/mcity.sumocfg',
    num_tries=10,
    gui_flag=False,
    output_path=f"{args.dir}/{args.mode}/{args.mode}_{args.nth}",
    sumo_output_file_types=["fcd_all", "collision", "tripinfo"],
)
monitor.bind_env(env)
sim.bind_env(env)
sim.run()