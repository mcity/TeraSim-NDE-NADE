#!/bin/bash
sleep 3

# temporary directory
DIR_NAME="/home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/output"
export HAS_LIBSUMO=1

echo $HOSTNAME
VERSION=v5
mode=cosim_test_aws_${VERSION}_${HOSTNAME}

mkdir -p ${DIR_NAME}/${mode}
mkdir -p ${DIR_NAME}/${mode}/raw_data
mkdir -p ${DIR_NAME}/${mode}/raw_data/final_state
mkdir -p ${DIR_NAME}/${mode}/raw_data/maneuver_challenges

del_mode="all"

cd /home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test

for i in {1..1000}; do
    mkdir -p ${DIR_NAME}/${mode}/raw_data/${mode}_0_${i}
    mkdir -p ${DIR_NAME}/${mode}/raw_data/cav_context_0_${i}
    
    redis-cli set launch_autoware 1
    redis-cli set iteration ${i}
    redis-cli set hostname $HOSTNAME
    redis-cli set version $VERSION
    
    echo "initializing autoware, waiting for 45 seconds..."
    sleep 45
    
    python3 safetest_mcity_cosim_main.py --dir ${DIR_NAME} --name ${mode} --nth 0_${i}

    redis-cli set launch_autoware 0

    echo "shuting down autoware, waiting for 45 seconds..."
    sleep 45

    echo "uploading file to aws cloud..."
    python3 upload_output_aws.py --dir ${DIR_NAME} --mode ${mode} --nth 0_${i}
done
