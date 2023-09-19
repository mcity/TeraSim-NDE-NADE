import numpy as np
import xml.etree.ElementTree as ET

def location_with_timestamp_extract_from_xml(xml_file_path, vehicle_id):
    tree = ET.parse(xml_file_path)
    root = tree.getroot()
    vehicle_locations = []

    timestampFound = False
    for timestep in root.findall('timestep'):
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

# if __name__ == "__main__":
#     fcd_file_name = "fcd_all.xml"
#     vehicle_id = "CAV"
#     locations = location_with_timestamp_extract_from_xml(fcd_file_name, vehicle_id)
#     print(get_vehicle_distance_from_location_with_timestamp(locations))