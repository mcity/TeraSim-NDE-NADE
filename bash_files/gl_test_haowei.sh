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
cd /home/haoweis/Safe-Test-TeraSim
module load python/3.10.4
source /home/haoweis/Safe-Test-TeraSim/venv/bin/activate

DIR_NAME="/scratch/henryliu_root/henryliu98/shared_data/safetest-nade"
export HAS_LIBSUMO=1

mode="ITE_calibration_change_2roundabout_out_priority_change_map"
mkdir -p ${DIR_NAME}/${mode}
mkdir -p ${DIR_NAME}/${mode}/raw_data
mkdir -p ${DIR_NAME}/${mode}/raw_data/final_state
mkdir -p ${DIR_NAME}/${mode}/raw_data/maneuver_challenges

del_mode="all"

for i in {1..200}; do
    mkdir -p ${DIR_NAME}/${mode}/raw_data/${mode}_${SLURM_ARRAY_TASK_ID}_${i}
    # test record
    python safetest_mcity_main.py --dir ${DIR_NAME} --mode ${mode} --nth ${SLURM_ARRAY_TASK_ID}_${i} > ${DIR_NAME}/${mode}/raw_data/${mode}_${SLURM_ARRAY_TASK_ID}_${i}/res.txt
    # remove if no collision with del_mode
    if [ $(grep "collisions" ${DIR_NAME}/${mode}/raw_data/${mode}_${SLURM_ARRAY_TASK_ID}_${i}/collision.xml -m 1 | wc -l) -eq 0 ]; then
        echo "Preserve ${DIR_NAME}/${mode}/raw_data/${mode}_${SLURM_ARRAY_TASK_ID}_${i}"
    else
        if [ $(grep "victim" ${DIR_NAME}/${mode}/raw_data/${mode}_${SLURM_ARRAY_TASK_ID}_${i}/collision.xml -m 1 | wc -l) -eq 0 ]; then
            echo "Removing ${DIR_NAME}/${mode}/raw_data/${mode}_${SLURM_ARRAY_TASK_ID}_${i}"
            # if del_mode is verbose, only remove fcd_all.xml
            if [ "${del_mode}" = "verbose" ]; then
                rm -f ${DIR_NAME}/${mode}/raw_data/${mode}_${SLURM_ARRAY_TASK_ID}_${i}/fcd_all.xml
            else
                rm -rf ${DIR_NAME}/${mode}/raw_data/${mode}_${SLURM_ARRAY_TASK_ID}_${i}
            fi
        fi
    fi
done
