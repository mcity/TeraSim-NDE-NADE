#!/bin/bash

#SBATCH --job-name="safetest_ite"
#SBATCH --mail-user=haoweis@umich.edu
#SBATCH --mail-type=BEGIN,END,FAIL
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=7gb
#SBATCH --array=0-499 # how many workers you are using
#SBATCH --time=00-02:00:00 # time duration
#SBATCH --account=henryliu98
#SBATCH --partition=standard
#SBATCH --output=/home/haoweis/safe_test.log # change to your directory



ulimit -c 0
cd /home/haoweis
source .bashrc
cd /home/haoweis/TeraSim-NDE-ITE
module load python/3.10.4
source /home/haoweis/TeraSim-NDE-ITE/venv/bin/activate
cd /home/haoweis/TeraSim-NDE-ITE/example

DIR_NAME="/scratch/henryliu_root/henryliu98/shared_data/safetest-nade"
export HAS_LIBSUMO=1

# add time stamp to experiment name
experiment_name="ITE_refactor_multiple_collision_avoid_with_log_accept_collision"
mkdir -p ${DIR_NAME}/${experiment_name}
mkdir -p ${DIR_NAME}/${experiment_name}/raw_data
mkdir -p ${DIR_NAME}/${experiment_name}/raw_data/final_state
mkdir -p ${DIR_NAME}/${experiment_name}/raw_data/maneuver_challenges
mkdir -p ${DIR_NAME}/${experiment_name}/raw_data/critical_moment_infos

del_mode="all"

for i in {1..200}; do
    exp_nth=${SLURM_ARRAY_TASK_ID}_${i}
    mkdir -p ${DIR_NAME}/${experiment_name}/raw_data/${exp_nth}
    # test record
    python safetest_mcity_main.py --dir ${DIR_NAME} --name ${experiment_name} --nth ${exp_nth} > ${DIR_NAME}/${experiment_name}/raw_data/${exp_nth}/res.txt
    # remove if no collision happens (no victim) or the victim is in the wrong junction
    if [ $(grep "victim" ${DIR_NAME}/${experiment_name}/raw_data/${exp_nth}/collision.xml -m 1 | wc -l) -eq 0 ] || [ $(grep "nd_34_1_6" ${DIR_NAME}/${experiment_name}/raw_data/${exp_nth}/collision.xml -m 1 | wc -l) -gt 0 ]; then
        echo "Removing ${DIR_NAME}/${experiment_name}/raw_data/${exp_nth}"
        if [ "${del_mode}" = "verbose" ]; then
            rm -f ${DIR_NAME}/${experiment_name}/raw_data/${exp_nth}/fcd_all.xml
        else
            rm -rf ${DIR_NAME}/${experiment_name}/raw_data/${exp_nth}
        fi
    fi
done