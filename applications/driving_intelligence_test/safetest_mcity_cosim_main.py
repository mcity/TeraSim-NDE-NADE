import sys
import os
file_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(file_path + "/../../")

from terasim.simulator import Simulator
from envs.env_monitor import EnvMonitor
from envs.safetest_nade_cosim import SafeTestNADECoSim
from terasim.logger.infoextractor import InfoExtractor
from mr_vehicle.mr_vehicle_factory import MRVehicleFactory

import redis
try:
    r = redis.Redis()
    keys = r.keys('*')
    r.delete(*keys)
except:
    pass

import argparse

def main(args):

    highlight_routes = set(["7", "10", "13", "14", "16"])
    monitor = EnvMonitor(
        highlight_routes=highlight_routes,
        log_dir=f"{args.dir}/{args.name}/raw_data",
        exp_id=f"{args.name}_{args.nth}",
    )

    veh_factory = MRVehicleFactory()
    env = SafeTestNADECoSim(
        vehicle_factory = veh_factory,
        info_extractor=InfoExtractor,
    )
    sim = Simulator(
        sumo_net_file_path = file_path + '/maps/Mcity_safetest/mcity.net.xml',
        sumo_config_file_path = file_path + '/maps/Mcity_safetest/mcity.sumocfg',
        num_tries=10,
        gui_flag=False,
        output_path=f"{args.dir}/{args.name}/raw_data/{args.name}_{args.nth}",
        sumo_output_file_types=["fcd", "collision"],
        realtime_flag=True,
        additional_sumo_args=["--start","--quit-on-end"]
    )
    monitor.bind_env(env)
    sim.bind_env(env)
    sim.run()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run simulation.')
    parser.add_argument('--dir', type=str, help='output directory', default="output")
    parser.add_argument('--name', type=str, help='experiment name.', default="test")
    parser.add_argument('--nth', type=str, help='the nth epoch', default="0_0")
    args = parser.parse_args()
    r = redis.Redis()
    r.flushdb()
    main(args)


