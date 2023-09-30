#!/bin/bash

# temporary directory
DIR_NAME="/home/mtl12345/terasim/TeraSim-NDE-ITE/output"
export HAS_LIBSUMO=1

mode="cosim_test_local"

mkdir -p ${DIR_NAME}/${mode}
mkdir -p ${DIR_NAME}/${mode}/raw_data
mkdir -p ${DIR_NAME}/${mode}/raw_data/final_state
mkdir -p ${DIR_NAME}/${mode}/raw_data/maneuver_challenges

del_mode="all"

for i in {1..50}; do
    mkdir -p ${DIR_NAME}/${mode}/raw_data/${mode}_0_${i}
    python3 safetest_mcity_cosim_main.py --dir ${DIR_NAME} --name ${mode} --nth 0_${i}
done
