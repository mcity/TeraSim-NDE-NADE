import os
import argparse
import pandas as pd
import xml.etree.ElementTree as ET
from tqdm import tqdm
import json
import xml.etree.ElementTree as ET
from shapely.geometry import Point, Polygon, LineString
import numpy as np
import math
from vehicle.vehicle_utils import get_location
# from vehicle.vehicle_utils import get_location
VEH_LENGTH, VEH_WIDTH, VEH_HEIGHT = 5.0, 1.85, 1.5

# @profile
def get_basic_info(path_name):
    # read the result file
    df = pd.read_csv(f"{path_name}/res.txt", header=None, names=["category", "veh_1", "veh_2", "reason", "time"], sep="\t")
    fcd_path = f"{path_name}/fcd_all.xml"
    monitor_json_path = f"{path_name}/monitor.json"
    df["time"] = df["time"].astype(float)

    crash_mark = df["category"] == "collision"
    timeout_mark = df["category"] == "timeout"
    end_time, crash_veh_1, crash_veh_2, importance = 0.0, "", "", 0.0
    neg_veh, neg_time_diff, neg_reason = "", -1.0, ""
    if crash_mark.sum() != 0:
        end_time = df["time"][crash_mark].iloc[-1]
        crash_veh_1 = df["veh_1"][crash_mark].iloc[-1]
        crash_veh_2 = df["veh_2"][crash_mark].iloc[-1]
        try:
            importance = float(df["reason"][crash_mark].iloc[-1])
        except:
            importance = 0.0
        
        # get the most recent negligence
        neg_mark = (df["category"] == "negligence") \
            & ((df["veh_1"] == crash_veh_1) | (df["veh_1"] == crash_veh_2)) \
            & (df["time"] < end_time)
        if neg_mark.sum() != 0:
            neg_veh = df["veh_1"][neg_mark].iloc[-1]
            neg_time_diff = end_time - df["time"][neg_mark].iloc[-1]
            neg_reason = df["reason"][neg_mark].iloc[-1]
    elif timeout_mark.sum() >= 1:
        # remember to use the default end time
        end_time = 1200.0
    else:
        end_time = 0.0

    mark = df["category"] == "intersection"
    maneuver_challenge = df["time"][mark].nunique()
    if crash_mark.sum() != 0:
        try:
            collision_type, collison_severity, collision_location, _, _ = get_collision_type_severity_from_json(fcd_path, monitor_json_path, crash_veh_1, crash_veh_2)
        except Exception as e:
            collision_type, collision_location = "", ""
            if os.path.exists(monitor_json_path):
                print(f"Error in getting collision type from fcd: {fcd_path}, {e}")
    else:
        collision_type, collision_location = "", ""
    return end_time, crash_veh_1, crash_veh_2, importance, maneuver_challenge, neg_veh, neg_time_diff, neg_reason, collision_type, collision_location


def get_route_length(path_name):
    tree = ET.parse(f"{path_name}/tripinfo.xml")
    # get all the tripinfo
    root = tree.getroot()
    tripinfo = root.findall("tripinfo")

    # sum all the routeLength in each tripinfo
    route_length = 0.0
    for trip in tripinfo:
        route_length += float(trip.get("routeLength"))
    return route_length

def get_relative_position(v1, v2):
    anchor_center_pt = [v1.location.x, v1.location.y]
    pt1, pt2, pt3, pt4 = v1.realworld_4_vertices  # upper left clockwise

    determine_v_center = Point(v2.location.x, v2.location.y)

    relative_position = None
    # front
    front_area = Polygon(_get_region_coordinates(anchor_center_pt, pt1, pt2, distance=100))
    if front_area.contains(determine_v_center):
        relative_position = 'front'
        return relative_position

    # rear
    rear_area = Polygon(_get_region_coordinates(anchor_center_pt, pt3, pt4, distance=100))
    if rear_area.contains(determine_v_center):
        relative_position = 'rear'
        return relative_position

    # left
    left_area = Polygon(_get_region_coordinates(anchor_center_pt, pt1, pt4, distance=100))
    if left_area.contains(determine_v_center):
        relative_position = 'left'
        return relative_position

    # right
    right_area = Polygon(_get_region_coordinates(anchor_center_pt, pt2, pt3, distance=100))
    if right_area.contains(determine_v_center):
        relative_position = 'right'
        return relative_position

def get_relative_heading(v1, v2):
    """
    Return the relative angle between two vehicles heading.
    Return in degrees in [0, 180].
    """
    angle = abs(v1.speed_heading - v2.speed_heading) % 360
    relative_heading = angle if (0 <= angle <= 180) else 360. - angle
    assert (0. <= relative_heading <= 180.)
    return relative_heading

def _get_region_coordinates(anchor_center_pt, direction_pt1, direction_pt2, distance=100):
    direction1 = np.arctan2(direction_pt1[1] - anchor_center_pt[1], direction_pt1[0] - anchor_center_pt[0])
    new_pt1 = [anchor_center_pt[0] + distance * np.cos(direction1), anchor_center_pt[1] + distance * np.sin(direction1)]

    direction2 = np.arctan2(direction_pt2[1] - anchor_center_pt[1], direction_pt2[0] - anchor_center_pt[0])
    new_pt2 = [anchor_center_pt[0] + distance * np.cos(direction2), anchor_center_pt[1] + distance * np.sin(direction2)]

    return anchor_center_pt, new_pt1, new_pt2

def get_collision_type(location_region, collided_position, relative_heading, veh1, veh2):
    collision_type = 'other'
    # rear end collisions are identified when "front" and "rear" in collided_position, and the degree is less than 90
    collided_position_set = set(collided_position[0] + collided_position[1])
    if 'front' in collided_position_set and 'rear' in collided_position_set and relative_heading < 60:
        collision_type = 'rear_end'
    # head on collisions are identified with 2 "front" in collided_position, and the degree is larger than 90
    elif "front" in collided_position[0] and "front" in collided_position[1] and relative_heading > 120:
        collision_type = 'head_on'
    # sideswipe collision and angle collision will only happen if one vehicle is only collided on the left/right (arbitrary other vehicle)
    elif "left" in collided_position_set or "right" in collided_position_set:
        if relative_heading < 20:
            collision_type = 'sideswipe'
        else:
            collision_type = 'angle'
    return collision_type

def get_last_timestep(monitor_json):
    # detect collision at each timestep, and return the first timestep with collision
    for timestep in monitor_json.keys():
        if get_collision_from_one_timestep(monitor_json, timestep):
            return timestep
    return list(monitor_json.keys())[-1]

def get_collision_from_one_timestep(monitor_json, timestep):
    vehicle_list = []
    for vid in monitor_json[timestep]:
        vehicle_json = monitor_json[timestep][vid]["obs"]["Ego"]
        x, y, road_id, heading, speed, v_width, v_length, v_height = vehicle_json["position"][0], vehicle_json["position"][1], vehicle_json["road_id"], -float(vehicle_json["heading"]) + 90., vehicle_json["velocity"], VEH_WIDTH, VEH_LENGTH, VEH_HEIGHT
        vehicle_tmp = construct_v_object(vid=vid, x=x, y=y, lane_id=road_id, heading=heading, speed=speed, v_width=VEH_WIDTH, v_length=VEH_LENGTH, v_height=VEH_HEIGHT, mass=1)
        vehicle_list.append(vehicle_tmp)
    if len(vehicle_list) < 2:
        return False
    else:
        v1_area = Polygon(vehicle_list[0].realworld_4_vertices)
        v2_area = Polygon(vehicle_list[1].realworld_4_vertices)
        if v1_area.intersects(v2_area):
            return True


def get_vehicle_from_json(monitor_json, last_timestep, colli_veh_1, colli_veh_2):
    veh1, veh2 = None, None

    for vid in monitor_json[last_timestep]:
        vehicle_json = monitor_json[last_timestep][vid]["obs"]["Ego"]
        x, y, road_id, heading, speed, _, _, _ = vehicle_json["position"][0], vehicle_json["position"][1], vehicle_json["road_id"], -float(vehicle_json["heading"]) + 90., vehicle_json["velocity"], VEH_WIDTH, VEH_LENGTH, VEH_HEIGHT
        if vid == colli_veh_1:
            veh1 = construct_v_object(vid=vid, x=x, y=y, lane_id=road_id, heading=heading, speed=speed, v_width=VEH_WIDTH, v_length=VEH_LENGTH, v_height=VEH_HEIGHT, mass=1)
        elif vid == colli_veh_2:
            veh2 = construct_v_object(vid=vid, x=x, y=y, lane_id=road_id, heading=heading, speed=speed, v_width=VEH_WIDTH, v_length=VEH_LENGTH, v_height=VEH_HEIGHT, mass=1)

    return veh1, veh2, road_id


def get_distance_from_json(monitor_json, last_timestep, colli_veh_1, colli_veh_2):
    mode_list = ["Lead", "LeftFoll", "RightFoll"]
    for mode, obs in monitor_json[last_timestep][colli_veh_1]["obs"].items():
        if mode in mode_list and obs["veh_id"] == colli_veh_2:
            distance = obs["distance"]
            return distance
    for mode, obs in monitor_json[last_timestep][colli_veh_2]["obs"].items():
        if mode in mode_list and obs["veh_id"] == colli_veh_1:
            distance = obs["distance"]
            return distance
    return -1


# @profile
def get_collision_type_severity_from_json(basic_infos, fcd_xml_path, monitor_json_path, colli_veh_1, colli_veh_2):
    # read the fcd file
    monitor_json = json.load(open(monitor_json_path, "r"))
    # last_timestep = sorted(monitor_json.keys())[-1]
    last_timestep = get_last_timestep(monitor_json)
    
    # fcd_obj = ET.parse(fcd_xml_path)
    # fcd_root = fcd_obj.getroot()
    
    veh1, veh2, road_id = get_vehicle_from_json(monitor_json, last_timestep, colli_veh_1, colli_veh_2)
    last_json_timestamp = list(monitor_json.keys())[-1]
    last_veh1, last_veh2, _ = get_vehicle_from_json(monitor_json, last_json_timestamp, colli_veh_1, colli_veh_2)
    
    location_region = get_location(road_id, lane_config=json.load(open("vehicle/lane_config.json", "r")))
    if basic_infos and "negligence_info" in basic_infos and basic_infos["negligence_info"] and "roundabout" in basic_infos["negligence_info"]:
        location_region = "roundabout"
    # get the relative position
    # relative_position = get_relative_position(veh1, veh2)
    collided_position = get_collided_position(veh1, veh2)
    # get the relative heading
    relative_heading = get_relative_heading(veh1, veh2)
    # get the collision type
    collision_type = get_collision_type(location_region, collided_position, relative_heading, veh1, veh2)
    collision_severity = get_collision_severity(last_veh1, last_veh2)
    if collision_type == "head_on" and get_relative_heading(last_veh1, last_veh2) < 100:
        collision_type = "angle"
    distance = get_distance_from_json(monitor_json, last_timestep, colli_veh_1, colli_veh_2)
    return collision_type, collision_severity, location_region, collided_position, relative_heading, distance

def get_collision_severity(v1_obj, v2_obj):
    v1_velocity, v1_heading = v1_obj.speed, v1_obj.speed_heading
    v2_velocity, v2_heading = v2_obj.speed, v2_obj.speed_heading
    v1_velocity_vec = np.array([v1_velocity * np.cos(math.radians(v1_heading)), v1_velocity * np.sin(math.radians(v1_heading))])
    v2_velocity_vec = np.array([v2_velocity * np.cos(math.radians(v2_heading)), v2_velocity * np.sin(math.radians(v2_heading))])
    speed_diff_norm = np.linalg.norm(v1_velocity_vec - v2_velocity_vec)
    return speed_diff_norm


def get_four_area_of_vehicle(v1):
    anchor_center_pt = [v1.location.x, v1.location.y]
    pt1, pt2, pt3, pt4 = v1.realworld_4_vertices  # upper left clockwise
    front_area = Polygon(_get_region_coordinates(anchor_center_pt, pt1, pt2))
    rear_area = Polygon(_get_region_coordinates(anchor_center_pt, pt3, pt4))
    left_area = Polygon(_get_region_coordinates(anchor_center_pt, pt1, pt4))
    right_area = Polygon(_get_region_coordinates(anchor_center_pt, pt2, pt3))
    return front_area, rear_area, left_area, right_area

def get_four_border_of_vehicle(v1):
    pt1, pt2, pt3, pt4 = v1.realworld_4_vertices  # upper left clockwise
    front_line = LineString([pt1, pt2])
    rear_line = LineString([pt3, pt4])
    left_line = LineString([pt1, pt4])
    right_line = LineString([pt2, pt3])
    return front_line, rear_line, left_line, right_line

def get_collided_position(v1, v2):
    v1_fron_area, v1_rear_area, v1_left_area, v1_right_area = get_four_area_of_vehicle(v1)
    v1_area_dict = {"front": v1_fron_area, "rear": v1_rear_area, "left": v1_left_area, "right": v1_right_area}
    v2_fron_area, v2_rear_area, v2_left_area, v2_right_area = get_four_area_of_vehicle(v2)
    v2_area_dict = {"front": v2_fron_area, "rear": v2_rear_area, "left": v2_left_area, "right": v2_right_area}
    v1_area = Polygon(v1.realworld_4_vertices)
    v2_area = Polygon(v2.realworld_4_vertices)
    # check if the areas of each vehicle is collided with another vehicle
    v1_collided_position_dict, v2_collided_position_dict = {}, {}
    for v1_area_name, v1_area_small in v1_area_dict.items():
        if v1_area_small.intersects(v2_area):
            collision_area_percentage = v1_area_small.intersection(v2_area).area / v1_area_small.area
            v1_collided_position_dict[v1_area_name] = collision_area_percentage
    for v2_area_name, v2_area_small in v2_area_dict.items():
        if v2_area_small.intersects(v1_area):
            collision_area_percentage = v2_area_small.intersection(v1_area).area / v2_area_small.area
            v2_collided_position_dict[v2_area_name] = collision_area_percentage
    # collided position is the area name with highest collision area percentage
    v1_collided_position_list = [k for k, v in v1_collided_position_dict.items()]
    v2_collided_position_list = [k for k, v in v2_collided_position_dict.items()]
    # v1_collided_position_list = filter_collided_position_list(v1_collided_position_list)
    # v2_collided_position_list = filter_collided_position_list(v2_collided_position_list)
    return v1_collided_position_list, v2_collided_position_list

def filter_collided_position_list(collided_position_list):
    if len(collided_position_list) >= 3 and "front" in collided_position_list and "rear" in collided_position_list:
        collided_position_list.remove("front")
        collided_position_list.remove("rear")
    return collided_position_list

def construct_v_object(vid, x, y, lane_id, heading, speed, v_width=1.8, v_length=4.8, v_height=1.5, mass=1):
    """
    The heading here is [deg] east:0, north:90, south:270
    """
    v = Vehicle()
    v.location.x, v.location.y = x, y
    v.id = vid
    v.speed_heading = heading
    v.speed = speed
    v.size = Size3d(width=v_width, length=v_length, height=v_height)
    v.update_poly_box_and_realworld_4_vertices()
    v.mass = mass
    v.lane_id = lane_id
    return v

# @profile
def upgraded_get_basic_info(path_name):
    collision_json_path = os.path.join(path_name, "final_state.json")
    basic_infos = json.load(open(collision_json_path, "r"))
    fcd_path = os.path.join(path_name, "fcd_all.xml")
    monitor_json_path = os.path.join(path_name, "monitor.json")
    
    end_reason = basic_infos["end_reason"]
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
    lane_id = basic_infos["veh_1_lane_id"] if "veh_1_lane_id" in basic_infos else basic_infos["lane_id"]
    # if basic_infos["veh_2_lane_id"] != lane_id:
    #     print(f"Error: {path_name}")
    if lane_id is None:
        lane_id = ""
    if neg_reason is None:
        neg_reason = ""
    if end_reason == "collision":
        try:
            collision_type, collision_severity, collision_location, _, _ = get_collision_type_severity_from_json(fcd_path, monitor_json_path, crash_veh_1, crash_veh_2)
        except Exception as e:
            collision_type, collision_location = "", ""
            if os.path.exists(monitor_json_path):
                print(f"Error in getting collision type from fcd: {fcd_path}, {e}")
            else:
                print(f"Error: {e}")
    else:
        collision_type, collision_location = "", "" 
    return end_time, crash_veh_1, crash_veh_2, importance, num_maneuver_challenges, neg_veh, neg_time_diff, neg_reason, collision_type, collision_location, route_length, lane_id
    

# @profile
def export_to_csv(path_name, export_file):
    # write the csv file
    with open(export_file, "w") as f:
        f.write("\t".join([
            "name", "end_time", 
            "crash_veh_1", "crash_veh_2", 
            "importance", "maneuver_challenge", 
            "neg_veh", "neg_time_diff", 
            "neg_reason",  
            "collision_type", "location_type",
            "route_length", "lane_id"
        ]) + "\n")
        info_cnt = 0
        with open("check.txt", "w") as c:
            for file in tqdm(os.listdir(path_name)):
                file_path = os.path.join(path_name, file)
                if os.path.isdir(file_path):
                    try:
                        # infos = [file, *get_basic_info(file_path)]
                        infos = [file, *upgraded_get_basic_info(file_path)]
                        f.write("\t".join([str(info) for info in infos]) + "\n")
                    except Exception as e:
                        collision_json_path = os.path.join(file_path, "final_state.json")
                        c.write(file_path + "\n")
                        if os.path.exists(collision_json_path):
                            print(f"Error in {file_path}: {e}")
                            info_cnt += 1
        print("info_error: ", info_cnt)


class Location(object):
    def __init__(self, x=None, y=None, z=None):
        self.x = x
        self.y = y
        self.z = z


class Size3d(object):
    def __init__(self, width=None, length=None, height=None):
        self.width = width
        self.length = length
        self.height = height


def _rotate_pt(x, y, a):
    return np.cos(a) * x - np.sin(a) * y, np.sin(a) * x + np.cos(a) * y


def get_box_pts_from_center_heading(length, width, xc, yc, heading):
    l, w = length / 2.0, width / 2.0

    ## box
    x1, y1 = l, w
    x2, y2 = l, -w
    x3, y3 = -l, -w
    x4, y4 = -l, w

    ## rotation
    a = heading / 180. * np.pi
    x1_, y1_ = _rotate_pt(x1, y1, a)
    x2_, y2_ = _rotate_pt(x2, y2, a)
    x3_, y3_ = _rotate_pt(x3, y3, a)
    x4_, y4_ = _rotate_pt(x4, y4, a)

    ## translation
    pt1 = [x1_ + xc, y1_ + yc]
    pt2 = [x2_ + xc, y2_ + yc]
    pt3 = [x3_ + xc, y3_ + yc]
    pt4 = [x4_ + xc, y4_ + yc]

    return [pt1, pt2, pt3, pt4]

def get_center_from_front_center(x_front_center, y_front_center, angle, VEH_LENGTH):
    def _rotate(x1, y1, x2, y2, angle=0.):
        x = (x1 - x2) * np.cos(angle) - (y1 - y2) * np.sin(angle) + x2
        y = (x1 - x2) * np.sin(angle) + (y1 - y2) * np.cos(angle) + y2
        return [x, y]

    x_center, y_center = _rotate(x_front_center - 0.5 * VEH_LENGTH, y_front_center, x_front_center, y_front_center, angle=-math.radians(angle - 90.))
    return x_center, y_center

class Vehicle(object):
    def __init__(self):
        "Configuration for vehicle states"
        self.location = Location()
        self.size = Size3d()
        self.realworld_4_vertices = None
        self.id = '-1'
        self.speed = None
        self.speed_heading = None
        self.poly_box = None  # The rectangle of the vehicle using shapely.Polygon
        self.gt_realworld_4_vertices = None  # using gt_size
        self.gt_poly_box = None  # using gt_size
        self.mass = None  # mass of the vehicle. To calculate the crash severity
        self.lane_id = None  # the lane id the vehicle is at

    def update_poly_box_and_realworld_4_vertices(self):
        """
        Update the poly box and realworld 4 vertices based on current location (x,y) and speed heading
        Returns
        -------

        """
        length, width = self.size.length, self.size.width
        realworld_4_vertices = get_box_pts_from_center_heading(length=length, width=width, xc=self.location.x, yc=self.location.y, heading=self.speed_heading)
        new_poly_box = Polygon(realworld_4_vertices)
        self.poly_box = new_poly_box
        self.realworld_4_vertices = np.array(realworld_4_vertices)

                    
# if __name__ == "__main__":
    # parser = argparse.ArgumentParser(description='Statistics.')
    # parser.add_argument('--path', type=str, help='path to directory', default="path/to/output")
    # parser.add_argument('--dir', type=str, help='output directory', default="output")
    # parser.add_argument('--mode', type=str, help='the negligence mode.', default="test")
    # args = parser.parse_args()

    # # use the following line if you want to use the default output directory
    # if args.path == "path/to/output":
    #     path_name=f"{args.dir}/{args.mode}"
    # else:
    #     path_name = args.path

    # export_file = f"stats.tsv"
    # export_to_csv(path_name, export_file)


    
    