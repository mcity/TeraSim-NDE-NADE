#!/bin/bash
export USE_LIBSUMO="0"
pkill roscore
redis-cli -h localhost -p 6379 flushall
# run the Autoware simulation
gnome-terminal --tab -- bash -c "python main_autoware_simulation.py" &
# run the SUMO simulation
# gnome-terminal --tab -- bash -c "python ABCwithterasim_mr.py" &
# run the ROS nodes
## CAV perception
gnome-terminal --tab -- bash -c "python examples/main_rosnode_av_perception.py" &
## CAV states
gnome-terminal --tab -- bash -c "python examples/main_rosnode_av_states.py" &
## CAV traffic lights detection
gnome-terminal --tab -- bash -c "python examples/main_rosnode_av_tls.py" &
## SUMO state
gnome-terminal --tab -- bash -c "python examples/main_rosnode_sumo_states.py --mode remote" &
gnome-terminal --tab -- bash -c "python examples/main_websocket_publish_terasim_status.py" &
gnome-terminal --tab -- bash -c "python examples/main_websocket_subscribe_terasim_status.py" &
## publish perception to websocket
gnome-terminal --tab -- bash -c "python examples/main_websocket_publish_av_perception.py" &
## publish states to websocket
gnome-terminal --tab -- bash -c "python examples/main_websocket_publish_av_states.py" &
## subscribe perception from websocket
gnome-terminal --tab -- bash -c "python examples/main_websocket_subscribe_av_perception.py" &
## subscribe states from websocket
gnome-terminal --tab -- bash -c "python examples/main_websocket_subscribe_av_states.py" &

