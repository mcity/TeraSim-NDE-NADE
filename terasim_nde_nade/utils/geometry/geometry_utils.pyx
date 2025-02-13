from libc.math cimport sqrt, pow, sin, cos, atan2, M_PI
cimport cython
import numpy as np
cimport numpy as np

# Constants
DEG_TO_RAD = M_PI / 180.0
DEFAULT_DISTANCE_THRESHOLD = 30.0

@cython.boundscheck(False)
@cython.wraparound(False)
cpdef double calculate_distance(double x1, double y1, double x2, double y2):
    """Calculate Euclidean distance between two points.
    
    Args:
        x1, y1: Coordinates of first point
        x2, y2: Coordinates of second point
    
    Returns:
        double: Euclidean distance
    """
    cdef double dx = x2 - x1
    cdef double dy = y2 - y1
    return sqrt(dx * dx + dy * dy)

@cython.boundscheck(False)
@cython.wraparound(False)
cpdef np.ndarray[double, ndim=2] get_circle_centers(np.ndarray[double, ndim=1] point, 
                                                   double agent_length,
                                                   str agent_type):
    """Calculate circle centers for collision detection.
    
    Args:
        point: Point coordinates and heading [x, y, heading]
        agent_length: Length of the agent
        agent_type: Type of agent ('vehicle' or other)
    
    Returns:
        np.ndarray: Array of circle center coordinates
    """
    cdef double heading = point[2]
    cdef double cos_heading = cos(heading)
    cdef double sin_heading = sin(heading)
    cdef double offset
    cdef np.ndarray[double, ndim=2] center_list

    if agent_type == 'vehicle':
        offset = agent_length / 3
        center_list = np.zeros((3, 2))
        
        # Front circle
        center_list[0, 0] = point[0] + offset * cos_heading
        center_list[0, 1] = point[1] + offset * sin_heading
        
        # Middle circle
        center_list[1, 0] = point[0]
        center_list[1, 1] = point[1]
        
        # Rear circle
        center_list[2, 0] = point[0] - offset * cos_heading
        center_list[2, 1] = point[1] - offset * sin_heading
    else:
        # Single circle for other agents
        center_list = np.zeros((1, 2))
        center_list[0, 0] = point[0]
        center_list[0, 1] = point[1]
    
    return center_list

@cython.boundscheck(False)
@cython.wraparound(False)
cpdef double calculate_circle_radius(double length, double width, str agent_type):
    """Calculate circle radius for collision detection based on agent type and dimensions.
    
    Args:
        length: Agent length
        width: Agent width
        agent_type: Type of agent ('vehicle' or other)
    
    Returns:
        double: Circle radius
    """
    if agent_type == "vehicle":
        return sqrt((length/6.0)**2 + (width/2.0)**2)
    else:
        return max(length, width) / 2.0 