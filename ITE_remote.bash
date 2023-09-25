#!/bin/bash
export USE_LIBSUMO="1"
pkill roscore
redis-cli -h localhost -p 6379 flushall

export MCITY_OCTANE_KEY=mcity
export MCITY_OCTANE_SERVER=https://atrium-simulation.um.city

# run the SUMO simulation
python safetest_mcity_cosim_main.py &
## publish perception to websocket
python examples/main_websocket_publish_av_perception.py &
## publish tls to websocket
python examples/main_websocket_publish_av_tls.py &
## publish terasim time to websocket
python examples/main_websocket_publish_terasim_time.py &
## subscribe states from websocket
python examples/main_websocket_subscribe_av_states.py &
## subscribe av planning time from websocket
python examples/main_websocket_subscribe_av_planning_time.py &

