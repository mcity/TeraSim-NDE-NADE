#!/bin/bash

cd /home/zhijie/autoware
. install/setup.bash

DIR_NAME="/home/zhijie/terasim/TeraSim-NDE-ITE/output"
mode="cosim_test_local"

while true
do
    # Use redis-cli to get the value of the key. Change mykey to your key.
    value=$(redis-cli get launch_autoware)
    iteration=$(redis-cli get iteration)
    
    if [[ $value -eq 1 ]]
    then
    	ros2 bag record -a --compression-mode file --compression-format zstd -o ${DIR_NAME}/${mode}/raw_data/rosbag_${iteration} 
    fi

    # Wait for a while before checking again.
    sleep 1
done
