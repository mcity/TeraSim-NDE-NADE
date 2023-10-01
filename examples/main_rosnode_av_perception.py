import yaml

from terasim_mr.communicationtools.redis2ros import ROSPublisher_BVafterPerceptionRedis

# Load the configuration file
with open("examples/configs/Mcity_abctest_param_cosim.yaml", 'r') as stream:
    configs = yaml.safe_load(stream)

# Create the ROS publisher to publish background vehicle information to ROS-based Autoware
ros_pub = ROSPublisher_BVafterPerceptionRedis(configs)
ros_pub.run()