#!/bin/bash

cd /home/zhijie/autoware
. install/setup.bash

while true
do
    # Use redis-cli to get the value of the key. Change mykey to your key.
    value=$(redis-cli get launch_autoware)
    
    if [[ $value -eq 1 ]]
    then
    	ros2 launch mcity_terasim cosim.launch.py
    fi

    # Wait for a while before checking again.
    sleep 1
done
