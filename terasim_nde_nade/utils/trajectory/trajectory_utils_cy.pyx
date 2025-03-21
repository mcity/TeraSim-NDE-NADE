cimport cython
from libc.math cimport M_PI, atan2, cos, pow, sin, sqrt

import numpy as np

cimport numpy as np

from scipy.interpolate import interp1d

# Constants
SUMO_TO_NORMAL_ANGLE_OFFSET = 90.0
DEG_TO_RAD = M_PI / 180.0

def sumo_trajectory_to_normal_trajectory(np.ndarray[double, ndim=2] sumo_trajectory, double veh_length=5.0):
    """Convert SUMO trajectory format to normal trajectory format.
    
    Args:
        sumo_trajectory: Trajectory in SUMO format
        veh_length: Vehicle length
    
    Returns:
        np.ndarray: Converted trajectory
    """
    cdef int n = sumo_trajectory.shape[0]
    cdef np.ndarray[double, ndim=2] normal_trajectory = np.empty((n, sumo_trajectory.shape[1]))
    cdef double rad

    for i in range(n):
        rad = (SUMO_TO_NORMAL_ANGLE_OFFSET - sumo_trajectory[i, 2]) * DEG_TO_RAD
        normal_trajectory[i, 2] = atan2(sin(rad), cos(rad))
        normal_trajectory[i, 0] = sumo_trajectory[i, 0] - veh_length / 2 * cos(normal_trajectory[i, 2])
        normal_trajectory[i, 1] = sumo_trajectory[i, 1] - veh_length / 2 * sin(normal_trajectory[i, 2])
        if sumo_trajectory.shape[1] > 3:
            normal_trajectory[i, 3:] = sumo_trajectory[i, 3:]
    return normal_trajectory

def interpolate_future_trajectory(np.ndarray[double, ndim=2] trajectory_list_array, 
                                double interpolate_resolution):
    """Interpolate trajectory with given resolution.
    
    Args:
        trajectory_list_array: Original trajectory array
        interpolate_resolution: Desired time resolution for interpolation
    
    Returns:
        np.ndarray: Interpolated trajectory
    """
    cdef np.ndarray[double, ndim=1] time_values = trajectory_list_array[:, -1]
    cdef np.ndarray[double, ndim=2] position_values = trajectory_list_array[:, :-1]

    # Create interpolation function
    interpolation_function = interp1d(time_values, position_values, axis=0, kind='linear')

    # Create new time values
    cdef double start_time = time_values[0]
    cdef double end_time = time_values[-1]
    cdef int num_points = int((end_time - start_time) / interpolate_resolution) + 1

    # Generate new time values with consistent intervals
    cdef np.ndarray[double, ndim=1] new_time_values = np.linspace(start_time, start_time + (num_points - 1) * interpolate_resolution, num_points)

    # Interpolate position values
    cdef np.ndarray[double, ndim=2] new_position_values = interpolation_function(new_time_values)

    # Combine time and position values
    return np.hstack((new_position_values, new_time_values[:, None])) 