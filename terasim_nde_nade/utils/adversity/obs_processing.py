from terasim.overlay import traci


def get_ff_acceleration(obs_dict):
    ff_speed = min(
        traci.vehicle.getFollowSpeed(
            obs_dict["ego"]["veh_id"],
            obs_dict["ego"]["velocity"],
            3000,
            obs_dict["ego"]["velocity"],
            7.06,
        ),
        traci.vehicle.getAllowedSpeed(obs_dict["ego"]["veh_id"]),
    )
    return (ff_speed - obs_dict["ego"]["velocity"]) / traci.simulation.getDeltaT()


def get_cf_acceleration(obs_dict, leader_info):
    leader_id, leader_distance = leader_info
    cf_speed_with_leading_vehicle = traci.vehicle.getFollowSpeed(
        obs_dict["ego"]["veh_id"],
        obs_dict["ego"]["velocity"],
        leader_distance,
        traci.vehicle.getSpeed(leader_id),
        7.06,
    )
    cf_acceleration = (
        cf_speed_with_leading_vehicle - obs_dict["ego"]["velocity"]
    ) / traci.simulation.getDeltaT()
    return cf_acceleration
