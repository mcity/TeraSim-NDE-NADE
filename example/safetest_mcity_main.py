from terasim.simulator import Simulator
from terasim_nde_ite.envs.env_monitor import EnvMonitor
from terasim_nde_ite.envs.safetest_nade import SafeTestNADE
from terasim.logger.infoextractor import InfoExtractor
from terasim_nde_ite.vehicle.safetest_vehicle_factory import SafetestVehicleFactory
import argparse
import os
dir_path = os.path.dirname(os.path.realpath(__file__))

parser = argparse.ArgumentParser(description='Run simulation.')
parser.add_argument('--dir', type=str, help='output directory', default="output")
parser.add_argument('--exp_name', type=str, help='experiment name', default="test")
parser.add_argument('--nth', type=str, help='the nth epoch', default="0_0")
args = parser.parse_args()

highlight_routes = {"7", "10", "13", "14", "16"}
monitor = EnvMonitor(
    highlight_routes=highlight_routes,
    log_dir=f"{args.dir}/{args.exp_name}/raw_data",
    exp_id=f"{args.exp_name}_{args.nth}",
)
env = SafeTestNADE(
    vehicle_factory = SafetestVehicleFactory(lane_config_path=dir_path+'/maps/Mcity_safetest/lane_config.json'),
    info_extractor=InfoExtractor,
)
sim = Simulator(
    sumo_net_file_path = dir_path+'/maps/Mcity_safetest/mcity.net.xml',
    sumo_config_file_path = dir_path+'/maps/Mcity_safetest/mcity.sumocfg',
    num_tries=10,
    gui_flag=True,
    output_path=f"{args.dir}/{args.exp_name}/raw_data/{args.exp_name}_{args.nth}",
    sumo_output_file_types=["fcd_all", "collision", "tripinfo"],
)
monitor.bind_env(env)
sim.bind_env(env)
sim.run()