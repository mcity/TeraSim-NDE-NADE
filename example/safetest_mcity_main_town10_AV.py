import argparse
from pathlib import Path

import hydra
from loguru import logger
from omegaconf import DictConfig
from terasim.logger.infoextractor import InfoExtractor
from terasim.simulator import Simulator

from terasim_nde_nade.envs import NADEWithAV
from terasim_nde_nade.vehicle import NDEVehicleFactory
from terasim_nde_nade.vru import NDEVulnerableRoadUserFactory

parser = argparse.ArgumentParser(description="Run simulation.")
parser.add_argument("--dir", type=str, help="output directory", default="output")
parser.add_argument("--name", type=str, help="experiment name", default="test")
parser.add_argument("--nth", type=str, help="the nth epoch", default="0_0")
parser.add_argument(
    "--aggregateddir", type=str, help="aggregated directory", default="aggregated"
)
args = parser.parse_args()

base_dir = Path(args.dir) / args.name / "raw_data" / args.nth
base_dir.mkdir(parents=True, exist_ok=True)

Path(args.aggregateddir).mkdir(parents=True, exist_ok=True)
aggregated_log_dir = Path(args.aggregateddir) / "loguru_run.log"

log_files = [base_dir / "loguru_run.log", aggregated_log_dir]
log_levels = ["TRACE", "INFO"]

for log_file, log_level in zip(log_files, log_levels):
    logger.add(
        log_file,
        # filter="terasim_nde_nade",
        level=log_level,
        enqueue=True,
        backtrace=True,
        serialize=True,
    )


@hydra.main(config_path="conf", config_name="config")
def main(cfg: DictConfig) -> None:
    assert "CAV_cfg" in cfg, "CAV_cfg is not in the config file"
    env = NADEWithAV(
        cav_cfg = cfg.CAV_cfg,
        vehicle_factory=NDEVehicleFactory(cfg=cfg),
        vru_factory=NDEVulnerableRoadUserFactory(cfg=cfg),
        info_extractor=InfoExtractor,
        log_flag=True,
        log_dir=base_dir,
        warmup_time_lb=300,
        warmup_time_ub=400,
        run_time=300,
    )

    dir_path = Path(__file__).parent
    sim = Simulator(
        sumo_net_file_path=dir_path / "maps" / "town10" / "town10.net.xml",
        sumo_config_file_path=dir_path / "maps" / "town10" / "town10.sumocfg",
        num_tries=10,
        gui_flag=True,
        output_path=base_dir,
        sumo_output_file_types=["fcd_all", "collision", "tripinfo"],
    )
    sim.bind_env(env)

    terasim_logger = logger.bind(name="terasim_nde_nade")
    terasim_logger.info(f"terasim_nde_nade: Experiment started {args.nth}")

    try:
        sim.run()
    except Exception as e:
        terasim_logger.exception(
            f"terasim_nde_nade: Running error catched, {e} at {args.nth} experiment"
        )


if __name__ == "__main__":
    main()
