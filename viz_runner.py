import analysis.stats_utils_v2 as stats_utils
from analysis.viz_utils_v2 import visualizer
import os
from tqdm import tqdm
import pandas as pd
import numpy as np

import argparse


def get_location_collision_type(exp_id, exp_stats_info):
    mark = exp_stats_info["name"] == exp_id
    location_type = exp_stats_info[mark]["location"].values[0]
    collision_type = exp_stats_info[mark]["collision"].values[0]
    return location_type, collision_type

def export_videos(export_path, file_name, map_location="."):
    if not os.path.exists(export_path):
        os.system(f"mkdir -p {export_path}")
        os.system(f"chmod 755 {export_path}")
    with open(file_name) as f:
        for line in tqdm(f.readlines()):
            exp_id = line.strip()
            path_name = os.path.join(experiment_record_repo, exp_id)
            if os.path.isdir(path_name):
                location_type, collision_type = get_location_collision_type(exp_id, exp_stats_info)
                neg_info = exp_stats_info[exp_stats_info["name"] == exp_id]["neg_info"].values[0]
                if location_type == "None" or collision_type == "None" or neg_info == "None":
                    visualizer(map_location, experiment_record_repo, exp_id, os.path.join(export_path, "None"))
                else:
                    visualizer(map_location, experiment_record_repo, exp_id, os.path.join(export_path, f"{neg_info}/{location_type}/{collision_type}"))


parser = argparse.ArgumentParser()
parser.add_argument("--file", type=str, default="check_list.txt")
parser.add_argument("--user", type=str, default="haoyang")
parser.add_argument("--mode", type=str, default="test")
parser.add_argument("--exp_repo", type=str, default="/scratch/henryliu_root/henryliu0/shared_data/safetest-nade")
parser.add_argument("--check", type=str, default="quick-check")

args = parser.parse_args()

# exp_repo = "/scratch/henryliu_root/henryliu0/shared_data/safetest-nade"
# mode="ITE_1e-7negligence_1e-2IS_disable_leftright_foll_negligence_ann_arbor_IDM_MOBIL_henryliu0"

exp_repo = args.exp_repo
mode = args.mode
user = args.user
check_file_name = args.file
check_mode=args.check

experiment_record_repo = f"{exp_repo}/{mode}/raw_data"
export_stat_dir = f"{exp_repo}/{mode}/{user}/processed_data"
export_path = f"{exp_repo}/{mode}/{user}/videos"
exp_stats_info = pd.read_csv(f"{export_stat_dir}/stats.tsv", sep='\t')                            
get_type_func = lambda x: x.split("_")[0] if x is not np.nan else "None"
exp_stats_info["location"] = list(map(get_type_func, exp_stats_info["location_type"]))
exp_stats_info["collision"] = list(map(get_type_func, exp_stats_info["collision_type"]))

check_list_path = f"{check_file_name}"
export_videos(f"{export_path}/{check_mode}", check_list_path)