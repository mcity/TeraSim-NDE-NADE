#!/bin/bash
export USE_LIBSUMO="1"
pkill roscore
redis-cli -h localhost -p 6379 flushall
# run the SUMO simulation
# gnome-terminal --tab -- bash -c "python safetest_mcity_cosim_main.py" &
## SUMO state
# gnome-terminal --tab -- bash -c "python examples/main_websocket_publish_terasim_status.py" &
## publish perception to websocket
gnome-terminal --tab -- bash -c "python examples/main_websocket_publish_av_perception.py" &
## publish tls to websocket
gnome-terminal --tab -- bash -c "python examples/main_websocket_publish_av_tls.py" &
## publish terasim time to websocket
gnome-terminal --tab -- bash -c "python examples/main_websocket_publish_terasim_time.py" &
## subscribe states from websocket
gnome-terminal --tab -- bash -c "python examples/main_websocket_subscribe_av_states.py" &
## subscribe av planning time from websocket
gnome-terminal --tab -- bash -c "python examples/main_websocket_subscribe_av_planning_time.py" &

