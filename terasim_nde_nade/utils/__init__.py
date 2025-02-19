"""TeraSim NDE/NADE utilities package."""

from .agents import (
    VehicleInfoForPredict,
    get_vehicle_info,
    is_car_following,
)
from .base.nde_command import NDECommand
from .base.types import AgentType, CommandType, VRUType
from .collision.collision_check_cy import check_collision
from .collision.collision_check_cy import check_trajectory_intersection as is_intersect
from .collision.collision_utils import (
    get_collision_type_and_prob,
    get_location,
    is_head_on,
)
from .geometry.geometry_utils_cy import get_circle_centers as get_circle_center_list_new
from .nade import (
    get_maneuver_challenge_environment,
    add_avoid_accept_collision_command,
    get_avoidability_dicts,
    modify_ndd_dict_according_to_avoidability,
    remove_collision_avoidance_command_using_avoidability,
    apply_collision_avoidance,
    get_criticality_dicts,
    get_ndd_distribution_from_ctx,
    update_ndd_distribution_to_vehicle_ctx,
    update_control_cmds_from_predicted_trajectory,
    adversarial_hook
)
from .trajectory.trajectory_predictor import (
    get_future_position_on_route,
    predict_future_trajectory_vehicle,
    predict_future_trajectory_environment,
)
from .trajectory.trajectory_utils_cy import (
    interpolate_future_trajectory,
    sumo_trajectory_to_normal_trajectory,
)

__all__ = [
    "AgentType",
    "VRUType",
    "CommandType",
    "NDECommand",
    "VehicleInfoForPredict",
    "predict_future_trajectory_vehicle",
    "predict_future_trajectory_environment",
    "check_collision",
    "sumo_trajectory_to_normal_trajectory",
    "interpolate_future_trajectory",
    "is_intersect",
    "get_circle_center_list_new",
    "get_collision_type_and_prob",
    "get_location",
    "is_head_on",
    "get_vehicle_info",
    "get_future_position_on_route",
]
