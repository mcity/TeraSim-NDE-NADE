#!/bin/bash
sleep 3

source /home/zhijie/autoware/install/setup.bash
DIR_NAME="/home/zhijie/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/output"
mode="cosim_test_local"

while true
do
    # Use redis-cli to get the value of the key. Change mykey to your key.
    value=$(redis-cli get launch_autoware)
    iteration=$(redis-cli get iteration)
    
    if [[ $value -eq 1 ]]
    then
        ros2 bag record --compression-mode file --compression-format zstd -o ${DIR_NAME}/${mode}/raw_data/rosbag_0_${iteration}\
        /tf\
        /localization/kinematic_state\
        /perception/object_recognition/objects\
        /perception/object_recognition/prediction/maneuver\
        /planning/scenario_planning/trajectory\
        /simulation/dummy_perception_publisher/object_info\
        /simulation/input/initialtwist\
        /vehicle/status/steering_status\
        /vehicle/status/velocity_status
    fi

    # Wait for a while before checking again.
    sleep 1
done
