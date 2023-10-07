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
import pathlib

from .stats_utils import get_collision_type_severity_from_json


def merge_each_core_json(path_name, export_file, suffix):
    merged_json = {}
    path_name = pathlib.Path(path_name)
    final_json_files = list(path_name.glob(f"**/*{suffix}"))
    for file in tqdm(final_json_files):
        json_file = os.path.join(path_name, file)
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

def get_collision_severity_from_fcd(fcd_root, colli_veh_1_id, colli_veh_2_id):
    # get the last timestep of the fcd root
    last_timestep = fcd_root[-1]
    # get the collision vehicle 1
    colli_veh_1 = None
    colli_veh_2 = None
    for vehicle in last_timestep:
        if vehicle.attrib["id"] == colli_veh_1_id:
            colli_veh_1 = vehicle
        if vehicle.attrib["id"] == colli_veh_2_id:
            colli_veh_2 = vehicle
    if colli_veh_1 is None or colli_veh_2 is None:
        return None
    # get the collision severity
    v1_velocity = float(colli_veh_1.attrib["speed"])
    v2_velocity = float(colli_veh_2.attrib["speed"])
    v1_heading = 90 - float(colli_veh_1.attrib["angle"]) # sumo angle is clockwise
    v2_heading = 90 - float(colli_veh_2.attrib["angle"])
    v1_heading = np.radians(v1_heading)
    v2_heading = np.radians(v2_heading)
    v1_velocity_vector = np.array([v1_velocity * np.cos(v1_heading), v1_velocity * np.sin(v1_heading)])
    v2_velocity_vector = np.array([v2_velocity * np.cos(v2_heading), v2_velocity * np.sin(v2_heading)])
    relative_velocity_vector = v1_velocity_vector - v2_velocity_vector
    relative_velocity = np.linalg.norm(relative_velocity_vector)
    return relative_velocity

def get_exp_collision(basic_infos, path_name, exp_id, crash_veh_1, crash_veh_2):
    path_name = pathlib.Path(path_name)
    fcd_path = list(path_name.glob(f"**/{exp_id}/fcd.xml"))[0]
    # fcd_tree = ET.parse(fcd_path)
    # fcd_root = fcd_tree.getroot()
    monitor_json_path = list(path_name.glob(f"**/{exp_id}/monitor.json"))[0] if len(list(path_name.glob(f"**/{exp_id}/monitor.json"))) > 0 else None
    # monitor_json_path = os.path.join(path_name, "monitor.json")
    end_reason = basic_infos["end_reason"]
    relative_heading = None
    distance = -10
    if end_reason == "collision":
        try:
            collision_type, collision_severity, collision_location, _, relative_heading, distance = get_collision_type_severity_from_json(basic_infos, fcd_path, monitor_json_path, crash_veh_1, crash_veh_2)
        except Exception as e:
            collision_type, collision_location, collision_severity = "", "", ""
            if os.path.exists(monitor_json_path):
                print(f"Error in getting collision type from fcd: {fcd_path}, {e}")
            else:
                print(f"Error: {e}")
    else:
        collision_type, collision_location, collision_severity = "", "", ""
    return collision_type, collision_location, relative_heading, distance, collision_severity


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
            "collision_type", "location_type", "relative_heading", "distance", "collision_severity"
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
                    infos = [exp_id, *get_exp_final_state(basic_infos), *get_exp_collision(basic_infos, f"{path_name}", f"{exp_id}", crash_veh_1, crash_veh_2)]
                    f.write("\t".join([str(info) for info in infos]) + "\n")
                except:
                    print(f"Error in {exp_id}")
                    info_cnt += 1
                    continue
        print("info_error: ", info_cnt)

from multiprocessing import Pool, cpu_count, Manager
from functools import partial
from tqdm.contrib.concurrent import process_map
def process_exp(exp_id, merged_final_state_json, path_name):
    if "_0_0" in exp_id:
        # print("skip: ", exp_id)
        return
    try:
        basic_infos = merged_final_state_json[exp_id]
        # manuever_json = merged_manuever_json[exp_id]
        crash_veh_1 = basic_infos["veh_1_id"]
        if crash_veh_1 is None:
            crash_veh_1 = ""
        crash_veh_2 = basic_infos["veh_2_id"]
        if crash_veh_2 is None:
            crash_veh_2 = ""
        infos = [exp_id, *get_exp_final_state(basic_infos), *get_exp_collision(basic_infos, f"{path_name}", f"{exp_id}", crash_veh_1, crash_veh_2)]
        return "\t".join([str(info) for info in infos]) + "\n"
    except Exception as e:
        # print(f"Error in {exp_id}: {e}")
        return None

def export_to_csv_aws(path_name, export_path):
    # write the csv file
    with open(f"{export_path}/stats.tsv", "w") as f:
        f.write("\t".join([
            "name", "end_time", 
            "crash_veh_1", "crash_veh_2", 
            "importance", "maneuver_challenge", 
            "neg_veh", "neg_time_diff", 
            "neg_reason", "neg_info", "route_length", "bv_22_route_length", "lane_id", 
            "collision_type", "location_type", "relative_heading", "distance", "collision_severity"
        ]) + "\n")
        info_cnt = 0
        
        # for server_dir in tqdm(os.listdir(path_name)):


        merged_final_state_json = merge_each_core_json(f"{path_name}", f"{export_path}/merged_final_state.json", "final_state.json")
        # merged_manuever_json = merge_each_core_json(f"{path_name}/maneuver_challenges", f"{export_path}/merged_manuever.json", "maneuver_challenges.json")
        
        with open("output.txt", "w") as c:
            with Manager() as manager:
                merged_final_state_json = manager.dict(merged_final_state_json)
                process_exp_partial = partial(process_exp, merged_final_state_json=merged_final_state_json, path_name=path_name)
                with Pool(cpu_count()) as p:
                    results = process_map(process_exp_partial, merged_final_state_json.keys(), max_workers=cpu_count())
                    for res in results:
                        if res is not None:
                            f.write(res)
