#!/bin/bash

# temporary directory
DIR_NAME="/home/zhijie/terasim/TeraSim-NDE-ITE/output"
export HAS_LIBSUMO=1

mode="cosim_test_local"

mkdir -p ${DIR_NAME}/${mode}
mkdir -p ${DIR_NAME}/${mode}/raw_data
mkdir -p ${DIR_NAME}/${mode}/raw_data/final_state
mkdir -p ${DIR_NAME}/${mode}/raw_data/maneuver_challenges

del_mode="all"

for i in {1..200}; do
    mkdir -p ${DIR_NAME}/${mode}/raw_data/${mode}_0_${i}
    redis-cli set launch_autoware 1
    redis-cli set iteration ${i}
    
    echo "initializing autoware, waiting for 30 seconds..."
    sleep 30
    
    python3 safetest_mcity_cosim_main.py --dir ${DIR_NAME} --name ${mode} --nth 0_${i}

    redis-cli set launch_autoware 0

    echo "shuting down autoware, waiting for 30 seconds..."
    sleep 30
done
