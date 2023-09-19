#!/bin/bash
pkill roscore
redis-cli -h localhost -p 6379 flushall
# run the Autoware simulation
gnome-terminal --tab -- bash -c "python main_autoware_simulation.py" &
# run the SUMO simulation
# gnome-terminal --tab -- bash -c "python ABCwithascs.py" &
# run the ROS nodes
## CAV perception
gnome-terminal --tab -- bash -c "python examples/main_rosnode_av_perception.py" &
## CAV states
gnome-terminal --tab -- bash -c "python examples/main_rosnode_av_states.py" &
## CAV traffic lights detection
gnome-terminal --tab -- bash -c "python examples/main_rosnode_av_tls.py" &
## SUMO state
gnome-terminal --tab -- bash -c "python examples/main_rosnode_sumo_states.py" &

gnome-terminal --tab -- bash -c "rviz" &


# then, kill the process of roscore
# then, clear redis server: redis-cli -h localhost -p 6379 flushall
