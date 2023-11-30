import json
import os, sys
sys.path.append("/media/mtl/2TB/ITE-refactor/TeraSim-NDE-ITE")
dir_path = "/media/mtl/2TB/ITE-refactor/TeraSim-NDE-ITE/example/"

from pathlib import Path
from terasim.simulator import Simulator
from terasim_nde_ite.envs.env_monitor import EnvMonitor
from terasim_nde_ite.envs.safetest_nade import SafeTestNADE
from terasim.logger.infoextractor import InfoExtractor
from terasim_nde_ite.vehicle.safetest_vehicle_factory import SafetestVehicleFactory
import argparse
parser = argparse.ArgumentParser(description='Run simulation.')
parser.add_argument('--dir', type=str, help='output directory', default="output")
parser.add_argument('--name', type=str, help='experiment name', default="test")
parser.add_argument('--nth', type=str, help='the nth epoch', default="0_0")
args = parser.parse_args()

monitor = EnvMonitor(
    log_dir=f"{args.dir}/{args.name}/raw_data",
    exp_id=f"{args.name}_{args.nth}",
)
env = SafeTestNADE(
    vehicle_factory = SafetestVehicleFactory(lane_config_path=dir_path+'/maps/Mcity_safetest/lane_config.json'),
    info_extractor=InfoExtractor,
)
sim = Simulator(
    sumo_net_file_path = dir_path+'/maps/Mcity_safetest/mcity.net.xml',
    sumo_config_file_path = dir_path+'/maps/Mcity_safetest/mcity.sumocfg',
    num_tries=10,
    gui_flag=False,
    output_path=f"{args.dir}/{args.name}/raw_data/{args.name}_{args.nth}",
    sumo_output_file_types=["fcd_all", "collision", "tripinfo"],
)
monitor.bind_env(env)
sim.bind_env(env)

sim.start()

control_cmds = {}
obs_dicts = {}
trajectory_dicts = {}

monitor_json_path = Path("/media/mtl/2TB/ITE-refactor/TeraSim-NDE-ITE/example/output/ITE_test_20231129-160821/raw_data/ITE_test_20231129-160821_0/monitor.json")

with open(monitor_json_path, "r") as f:
    monitor_json = json.load(f)

print(monitor_json.keys())

timestep = list(monitor_json.keys())[-1]
for veh_id in monitor_json[timestep]:
    control_cmds[veh_id] = monitor_json[timestep][veh_id]["cmd"]
    obs_dicts[veh_id] = monitor_json[timestep][veh_id]["obs"]
    trajectory_dicts[veh_id] = monitor_json[timestep][veh_id]["cmd"]["info"]["trajectory_prediction"]

control_commands = env.fix_intersection_decisions_helper(control_cmds, obs_dicts, trajectory_dicts, focus_id_list=list(control_cmds.keys()))

print("aaa")
