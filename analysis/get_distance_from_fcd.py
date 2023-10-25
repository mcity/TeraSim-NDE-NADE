import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path

def location_with_timestamp_extract_from_xml(fcd_root, vehicle_id):
    vehicle_locations = []
    timestampFound = False
    for timestep in fcd_root.findall('timestep'):
        time = timestep.get('time')
        vehicleFound = False
        for vehicle in timestep.findall('vehicle'):
            if vehicle.get('id') == vehicle_id:
                x = vehicle.get('x')
                y = vehicle.get('y')
                z = vehicle.get('z')
                vehicle_locations.append((time, x, y, z))
                vehicleFound = True
                break
        if vehicleFound and not timestampFound:
            timestampFound = True
        if timestampFound and not vehicleFound:
            break
    return np.array(vehicle_locations, dtype=float)

def get_vehicle_distance_from_location_with_timestamp(locations: np.ndarray):
    diffs = np.diff(locations[:, 1:], axis=0)
    distances = np.linalg.norm(diffs, axis=1)
    total_distance = np.sum(distances)
    return total_distance

def get_distance_from_fcd(fcd_path, vehicle_id):
    fcd_root = ET.parse(fcd_path).getroot()
    locations = location_with_timestamp_extract_from_xml(fcd_root, vehicle_id)
    return get_vehicle_distance_from_location_with_timestamp(locations)
from tqdm import tqdm
if __name__ == "__main__":
    data_folder = Path("./applications/driving_intelligence_test/output/cosim_test_local/raw_data")
    fcd_file_list = list(data_folder.glob("*/fcd.xml"))
    total_distance = 0
    for fcd_file_name in tqdm(fcd_file_list):
        vehicle_id = "CAV"
        # print(fcd_file_name, get_distance_from_fcd(fcd_file_name, vehicle_id))
        total_distance += get_distance_from_fcd(fcd_file_name, vehicle_id)
    print(total_distance)
