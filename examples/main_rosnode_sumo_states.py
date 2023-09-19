import argparse

from ascs.communicationtools import constants
from ascs.communicationtools.redis2ros import ROSPublisher_SUMOStateRedis


# Create the ROS publisher to publish the SUMO status to ROS-based Autoware
ros_pub = ROSPublisher_SUMOStateRedis()
ros_pub.run()
parser = argparse.ArgumentParser(description='ROS publisher for SUMO state')
parser.add_argument('--mode', type=str, default="local", help='running mode')
args = parser.parse_args()
# initialize the ROS node
if args.mode == "local":
    ros_publisher = ROSPublisher_SUMOStateRedis()
elif args.mode == "remote":
    ros_publisher = ROSPublisher_SUMOStateRedis(redis_key=constants.REDIS_CONSTANTS.TERASIM_STATUS_WEB_SUB)
else:
    raise ValueError("Invalid running mode!")
ros_publisher.run()