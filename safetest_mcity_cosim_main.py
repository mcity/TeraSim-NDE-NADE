from mtlsp.simulator import Simulator
from envs.env_monitor import EnvMonitor
from envs.safetest_nade_cosim import SafeTestNADECoSim
from mtlsp.logger.infoextractor import InfoExtractor
from vehicle.safetest_vehicle_factory import SafetestCosimVehicleFactory_Local, SafetestCosimVehicleFactory_Remote

import redis
try:
    r = redis.Redis()
    keys = r.keys('*')
    r.delete(*keys)
except:
    pass

import argparse

parser = argparse.ArgumentParser(description='Run simulation.')
parser.add_argument('--dir', type=str, help='output directory', default="output")
parser.add_argument('--mode', type=str, help='the negligence mode.', default="test")
parser.add_argument('--nth', type=str, help='the nth epoch', default="0_0")
parser.add_argument('--communication', type=str, help='the mode of communication', default="remote")
args = parser.parse_args()

highlight_routes = set(["7", "10", "13", "14", "16"])
monitor = EnvMonitor(
    highlight_routes=highlight_routes,
    log_dir=f"{args.dir}/{args.mode}/raw_data",
    exp_id=f"{args.mode}_{args.nth}",
)
if args.communication == "local":
    veh_factory = SafetestCosimVehicleFactory_Local()
elif args.communication == "remote":
    veh_factory = SafetestCosimVehicleFactory_Remote()
env = SafeTestNADECoSim(
    vehicle_factory = veh_factory,
    info_extractor=InfoExtractor,
)
sim = Simulator(
    sumo_net_file_path = './maps/Mcity/mcity_new.net.xml',
    sumo_config_file_path = './maps/Mcity/mcity_new.sumocfg',
    num_tries=10,
    gui_flag=True,
    output_path=f"{args.dir}/{args.mode}/raw_data/{args.mode}_{args.nth}",
    sumo_output_file_types=["fcd", "collision"],
    realtime_flag=True,
)
monitor.bind_env(env)
sim.bind_env(env)
sim.run()