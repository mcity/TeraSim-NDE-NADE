from terasim_mr.communicationtools.redis2ros import ROSSubsriber_ABCTests_POV_VRU_Redis


# Create the ROS publisher to publish the frontal traffic light signal information to ROS-based Autoware
# ros_pub = ROSPublisher_SUMOTrafficlight()
ros_pub = ROSSubsriber_ABCTests_POV_VRU_Redis()
ros_pub.run()