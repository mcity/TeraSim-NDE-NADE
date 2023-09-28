#!/bin/bash
export USE_LIBSUMO="1"
pkill roscore

export MCITY_OCTANE_KEY=mcity
export MCITY_OCTANE_SERVER=https://atrium-simulation.um.city

#Start redis Server
redis-server & sleep 5s
redis-cli -h localhost -p 6379 flushall &

## publish perception to McityOS
python examples/main_mcityos_publish_av_context.py &
## publish tls to McityOS
python examples/main_mcityos_publish_av_tls.py &
## publish terasim time to McityOS
python examples/main_mcityos_publish_terasim_time.py &
## subscribe av states from McityOS
python examples/main_mcityos_subscribe_av_states.py &
## subscribe av planning time from McityOS
python examples/main_mcityos_subscribe_av_planning_time.py &

