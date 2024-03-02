from typing import List, Tuple
cimport cython
@cython.boundscheck(False)
@cython.wraparound(False)
cpdef Tuple[Tuple[float, float], float] get_future_position_on_route(
        traci,
        str veh_edge_id, 
        float veh_lane_position, 
        int veh_lane_index, 
        str veh_lane_id,
        List[str] veh_route_id_list, 
        List[float] veh_route_length_list, 
        float future_distance, 
        int future_lateral_offset
    ):
    """
    Given the current vehicle edge id, lane position, current lane id, and the future distance / future lateral offset, predict the future position of the vehicle.
    """
    cdef:
        float current_lane_length
        int current_route_index
        int max_lane_index
        str vehicle_lane_id
        Tuple[float, float] future_position
        float future_heading

    veh_lane_position += future_distance
    current_lane_length = traci.lane.getLength(veh_lane_id)
    current_route_index = veh_route_id_list.index(veh_edge_id)

    # calculate the corresponding edge and lane position
    while veh_lane_position > current_lane_length and current_route_index < len(veh_route_id_list) - 1:
        current_route_index += 1
        veh_edge_id = veh_route_id_list[current_route_index]
        veh_lane_position -= current_lane_length
        current_lane_length = veh_route_length_list[current_route_index]

    # calculate the new lane index
    max_lane_index = traci.edge.getLaneNumber(veh_edge_id)
    veh_lane_index = min(max_lane_index, max(0, veh_lane_index + future_lateral_offset))
    vehicle_lane_id = veh_edge_id + "_" + str(veh_lane_index)
    future_position = traci.simulation.convert2D(veh_edge_id, veh_lane_position, veh_lane_index)
    future_heading = traci.lane.getAngle(vehicle_lane_id, veh_lane_position)
    return future_position, future_heading