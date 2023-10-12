#!/bin/bash

# temporary directory
DIR_NAME="/home/mtl/Haojie/TeraSim/Safe-Test-TeraSim/output"
export HAS_LIBSUMO=1

mode="cosim_test_localcomputer8"
mkdir -p ${DIR_NAME}/${mode}
mkdir -p ${DIR_NAME}/${mode}/raw_data
mkdir -p ${DIR_NAME}/${mode}/raw_data/final_state
mkdir -p ${DIR_NAME}/${mode}/raw_data/maneuver_challenges

del_mode="all"

for i in {1..1000}; do
    mkdir -p ${DIR_NAME}/${mode}/raw_data/${mode}_0_${i}
    # test record
    # python safetest_mcity_cosim_main_local.py --dir ${DIR_NAME} --mode ${mode} --nth 0_${i} > ${DIR_NAME}/${mode}/raw_data/${mode}_0_${i}/res.txt
    python upload_output_aws.py --dir ${DIR_NAME} --mode ${mode} --nth 0_${i}
done
