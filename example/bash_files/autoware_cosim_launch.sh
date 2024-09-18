#!/bin/bash
sleep 3

REDIS_CLI_CMD="redis-cli -h localhost -p 6379"

source /opt/ros/humble/setup.bash
source $HOME/autoware/install/setup.bash

while true
do
    # Use redis-cli to get the value of the key. Change mykey to your key.
    value=$($REDIS_CLI_CMD get av_status)
    
    if [[ $value -eq 1 ]]
    then
    	ros2 launch mcity_demo cosim.launch.py
    fi

    # Wait for a while before checking again.
    sleep 1
done
