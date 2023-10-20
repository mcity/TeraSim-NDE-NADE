#!/bin/bash
sleep 2

DIR_NAME="/home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/output"
export HAS_LIBSUMO=1

echo $HOSTNAME
VERSION=v5
mode=cosim_test_aws_${VERSION}_${HOSTNAME}

i=$(</home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/iteration.txt)
next=$((i+1))
echo $next > /home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/iteration.txt

mkdir -p ${DIR_NAME}/${mode}
mkdir -p ${DIR_NAME}/${mode}/raw_data
mkdir -p ${DIR_NAME}/${mode}/raw_data/final_state
mkdir -p ${DIR_NAME}/${mode}/raw_data/maneuver_challenges

del_mode="all"

cd /home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test

mkdir -p ${DIR_NAME}/${mode}/raw_data/${mode}_0_${i}
mkdir -p ${DIR_NAME}/${mode}/raw_data/cav_context_0_${i}

redis-cli set iteration ${i}
redis-cli set hostname $HOSTNAME
redis-cli set version $VERSION
redis-cli set launch_autoware 1

echo "initializing autoware, waiting for 45 seconds..."
sleep 45

python3 safetest_mcity_cosim_main.py --dir ${DIR_NAME} --name ${mode} --nth 0_${i}

# Get a list of ROS2 process IDs and kill them
pids=$(pgrep -f ros2)
for pid in $pids
do
    kill -INT $pid
done

redis-cli set launch_autoware 0
sleep 15

echo "uploading file to aws cloud..."
python3 upload_output_aws.py --dir ${DIR_NAME} --mode ${mode} --nth 0_${i}
echo "file uploading complete"

sleep 15
sudo reboot
