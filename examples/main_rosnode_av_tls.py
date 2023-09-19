from ascs.communicationtools.redis2ros import ROSPublisher_DummyTrafficlight,ROSPublisher_SUMOTrafficlight


# Create the ROS publisher to publish the frontal traffic light signal information to ROS-based Autoware
ros_pub = ROSPublisher_SUMOTrafficlight()
# ros_pub = ROSPublisher_DummyTrafficlight()
ros_pub.run()