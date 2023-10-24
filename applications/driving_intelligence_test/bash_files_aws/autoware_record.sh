#!/bin/bash
sleep 5

echo "start recording rosbag..."

source /home/ubuntu/autoware/install/setup.bash
DIR_NAME="/home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/output"

version=$(redis-cli get version)
mode=cosim_test_aws_${version}_${HOSTNAME}
iteration=$(redis-cli get iteration)

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