
from terasim.vehicle.sensors.ego import EgoSensor
from terasim.overlay import traci

class NDEEgoSensor(EgoSensor):
    DEFAULT_PARAMS = dict(
        fields = {
            'velocity': traci.vehicle.getSpeed,
            'position': traci.vehicle.getPosition,
            'position3d': traci.vehicle.getPosition3D,
            'heading': traci.vehicle.getAngle,
            'edge_id': traci.vehicle.getRoadID,
            "lane_id": traci.vehicle.getLaneID,
            'lane_index': traci.vehicle.getLaneIndex,
            'acceleration': traci.vehicle.getAcceleration,
            'next_links': traci.vehicle.getNextLinks,
        }
    )

    def fetch(self) -> dict:
        original_dict = super().fetch()
        next_links = original_dict['next_links']
        original_dict['upcoming_lanes'] = [original_dict['lane_id']]
        original_dict['upcoming_lanes'] += get_next_lane_id_set_from_next_links(next_links) # include current lane
        original_dict['upcoming_foe_lane_id_set'] = get_upcoming_foe_lane_id_set(original_dict['upcoming_lanes'])
        return original_dict
    
def get_upcoming_foe_lane_id_set(upcoming_lanes):
    veh1_foe_lane_id_set = set()
    for lane_id in upcoming_lanes:
        veh1_foe_lane_id_set = veh1_foe_lane_id_set.union(set(traci.lane.getInternalFoes(lane_id)))
    return veh1_foe_lane_id_set

def get_next_lane_id_set_from_next_links(next_links):
    if len(next_links) == 0:
        return []
    next_lane_id = next_links[0][0]
    via_lane_id = next_links[0][4]
    return [via_lane_id, next_lane_id]

