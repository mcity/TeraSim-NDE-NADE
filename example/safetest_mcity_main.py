from pathlib import Path
import argparse
from loguru import logger
from terasim.simulator import Simulator
from terasim_nde_nade.envs.safetest_nade import SafeTestNADE
from terasim.logger.infoextractor import InfoExtractor
from terasim_nde_nade.vehicle.nde_vehicle_factory import NDEVehicleFactory

parser = argparse.ArgumentParser(description="Run simulation.")
parser.add_argument("--dir", type=str, help="output directory", default="output")
parser.add_argument("--name", type=str, help="experiment name", default="test")
parser.add_argument("--nth", type=str, help="the nth epoch", default="0_0")
args = parser.parse_args()

base_dir = Path(args.dir) / args.name / "raw_data" / args.nth
base_dir.mkdir(parents=True, exist_ok=True)

logger.add(
    base_dir / "loguru_run.log",
    filter="terasim_nde_nade",
    level="INFO",
)

env = SafeTestNADE(
    vehicle_factory=NDEVehicleFactory(),
    info_extractor=InfoExtractor,
    log_flag=True,
    log_dir=base_dir,
    warmup_time_lb=900,
    warmup_time_ub=1200,
    run_time=60,
)

dir_path = Path(__file__).parent
sim = Simulator(
    sumo_net_file_path=dir_path / "maps" / "Mcity_safetest" / "mcity.net.xml",
    sumo_config_file_path=dir_path / "maps" / "Mcity_safetest" / "mcity.sumocfg",
    num_tries=10,
    gui_flag=False,
    output_path=base_dir,
    sumo_output_file_types=["fcd_all", "collision", "tripinfo"],
)
sim.bind_env(env)
sim.run()
