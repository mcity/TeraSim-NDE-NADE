import os
import argparse
import pandas as pd
import xml.etree.ElementTree as ET
from tqdm import tqdm
import json
import xml.etree.ElementTree as ET
from shapely.geometry import Point, Polygon
import numpy as np
import math
from vehicle.vehicle_utils import get_location
# from vehicle.vehicle_utils import get_location
VEH_LENGTH, VEH_WIDTH, VEH_HEIGHT = 5.0, 1.85, 1.5

from .stats_utils import get_collision_type_from_json


def merge_each_core_json(path_name, export_file, suffix):
    merged_json = {}
    for file in tqdm(os.listdir(path_name)):
        json_file = os.path.join(path_name, file)
        # if end with suffix
        if file.endswith(suffix):
            try:
                json_data = json.load(open(json_file, "r"))
                merged_json.update(json_data)
            except Exception as e:
                print(f"Error: {e} in {json_file}")
    with open(export_file, "w") as f:
        json.dump(merged_json, f, indent=4)
    return merged_json


def get_exp_final_state(basic_infos):
    end_time = basic_infos["end_time"]
    importance = basic_infos["importance"]
    crash_veh_1 = basic_infos["veh_1_id"]
    if crash_veh_1 is None:
        crash_veh_1 = ""
    crash_veh_2 = basic_infos["veh_2_id"]
    if crash_veh_2 is None:
        crash_veh_2 = ""
    num_maneuver_challenges = basic_infos["num_maneuver_challenges"]
    neg_veh = basic_infos["negligence_car"]
    if neg_veh is None:
        neg_veh = ""
    if basic_infos["negligence_time"] <= 900:
        neg_time_diff = -1
    else:
        neg_time_diff = basic_infos["end_time"] - basic_infos["negligence_time"]
    neg_reason = basic_infos["negligence_mode"]
    route_length = basic_infos["distance"]
    bv_22_route_length = basic_infos["bv_22_distance"]
    lane_id = basic_infos["veh_1_lane_id"] if "veh_1_lane_id" in basic_infos else basic_infos["lane_id"]
    # if basic_infos["veh_2_lane_id"] != lane_id:
    #     print(f"Error: {path_name}")
    if lane_id is None:
        lane_id = ""
    if neg_reason is None:
        neg_reason = ""
    neg_info = ""
    if basic_infos["negligence_info"] is not None:
        neg_info = basic_infos["negligence_info"]
    return end_time, crash_veh_1, crash_veh_2, importance, num_maneuver_challenges, neg_veh, neg_time_diff, neg_reason, neg_info, route_length, bv_22_route_length, lane_id
    

def get_exp_collision(basic_infos, path_name, crash_veh_1, crash_veh_2):
    fcd_path = os.path.join(path_name, "fcd_all.xml")
    monitor_json_path = os.path.join(path_name, "monitor.json")
    end_reason = basic_infos["end_reason"]
    relative_heading = None
    distance = -10
    if end_reason == "collision":
        try:
            collision_type, collision_location, _, relative_heading, distance = get_collision_type_from_json(basic_infos, fcd_path, monitor_json_path, crash_veh_1, crash_veh_2)
        except Exception as e:
            collision_type, collision_location = "", ""
            if os.path.exists(monitor_json_path):
                print(f"Error in getting collision type from fcd: {fcd_path}, {e}")
            else:
                print(f"Error: {e}")
    else:
        collision_type, collision_location = "", ""
    return collision_type, collision_location, relative_heading, distance


# @profile
def export_to_csv(path_name, export_path):
    # write the csv file
    with open(f"{export_path}/stats.tsv", "w") as f:
        f.write("\t".join([
            "name", "end_time", 
            "crash_veh_1", "crash_veh_2", 
            "importance", "maneuver_challenge", 
            "neg_veh", "neg_time_diff", 
            "neg_reason", "neg_info", "route_length", "bv_22_route_length", "lane_id", 
            "collision_type", "location_type", "relative_heading", "distance"
        ]) + "\n")
        info_cnt = 0
        
        merged_final_state_json = merge_each_core_json(f"{path_name}/final_state", f"{export_path}/merged_final_state.json", "final_state.json")
        # merged_manuever_json = merge_each_core_json(f"{path_name}/maneuver_challenges", f"{export_path}/merged_manuever.json", "maneuver_challenges.json")
        
        with open("check.txt", "w") as c:
            for exp_id in tqdm(merged_final_state_json.keys()):
                try:
                    basic_infos = merged_final_state_json[exp_id]
                    # manuever_json = merged_manuever_json[exp_id]
                    crash_veh_1 = basic_infos["veh_1_id"]
                    if crash_veh_1 is None:
                        crash_veh_1 = ""
                    crash_veh_2 = basic_infos["veh_2_id"]
                    if crash_veh_2 is None:
                        crash_veh_2 = ""
                    infos = [exp_id, *get_exp_final_state(basic_infos), *get_exp_collision(basic_infos, f"{path_name}/{exp_id}", crash_veh_1, crash_veh_2)]
                    f.write("\t".join([str(info) for info in infos]) + "\n")
                except:
                    print(f"Error in {exp_id}")
                    info_cnt += 1
                    continue
        print("info_error: ", info_cnt)