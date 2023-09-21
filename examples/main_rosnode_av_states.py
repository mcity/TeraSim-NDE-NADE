import yaml

from terasim-mr.communicationtools.redis2ros import ROSSubsriber_AVAutowareRedis

# Load the configuration file
with open("examples/configs/Mcity_abctest_param_cosim.yaml", 'r') as stream:
    configs = yaml.safe_load(stream)

# Create the ROS subscriber to get CAV states from ROS-based Autoware
ros_pub = ROSSubsriber_AVAutowareRedis(configs)
ros_pub.run()