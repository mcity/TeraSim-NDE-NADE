#!/bin/bash
export USE_LIBSUMO="1"
pkill roscore

export MCITY_OCTANE_KEY=mcity
export MCITY_OCTANE_SERVER=https://atrium-simulation.um.city

# run the SUMO simulation
experiment_name="ITE_test_$(date +%Y%m%d-%H%M%S)"

# set DIR_name with experiment_name and time
DIR_NAME="./output"

# print the experiment name
echo "Experiment name: ${experiment_name}"

mkdir -p ${DIR_NAME}/${experiment_name}
mkdir -p ${DIR_NAME}/${experiment_name}/raw_data
mkdir -p ${DIR_NAME}/${experiment_name}/raw_data/final_state
mkdir -p ${DIR_NAME}/${experiment_name}/raw_data/maneuver_challenges

for i in {1..1000}; do
    mkdir -p ${DIR_NAME}/${experiment_name}/raw_data/${i}
    # test record
    python3 safetest_mcity_cosim_main.py --dir ${DIR_NAME} --name ${experiment_name} --nth ${i} > ${DIR_NAME}/${experiment_name}/raw_data/${i}/res.txt
done
