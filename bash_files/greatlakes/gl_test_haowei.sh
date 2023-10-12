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

exp_name="ITE_autoware_universe_map_recalibrate"
mkdir -p ${DIR_NAME}/${exp_name}
mkdir -p ${DIR_NAME}/${exp_name}/raw_data
mkdir -p ${DIR_NAME}/${exp_name}/raw_data/final_state
mkdir -p ${DIR_NAME}/${exp_name}/raw_data/maneuver_challenges

del_mode="all"

for i in {1..200}; do
    mkdir -p ${DIR_NAME}/${exp_name}/raw_data/${exp_name}_${SLURM_ARRAY_TASK_ID}_${i}
    # test record
    python applications/driving_intelligence_test/safetest_mcity_main.py --dir ${DIR_NAME} --exp_name ${exp_name} --nth ${SLURM_ARRAY_TASK_ID}_${i} > ${DIR_NAME}/${exp_name}/raw_data/${exp_name}_${SLURM_ARRAY_TASK_ID}_${i}/res.txt
    # remove if no collision with del_mode
    if [ $(grep "collisions" ${DIR_NAME}/${exp_name}/raw_data/${exp_name}_${SLURM_ARRAY_TASK_ID}_${i}/collision.xml -m 1 | wc -l) -eq 0 ]; then
        echo "Preserve ${DIR_NAME}/${exp_name}/raw_data/${exp_name}_${SLURM_ARRAY_TASK_ID}_${i}"
    else
        if [ $(grep "victim" ${DIR_NAME}/${exp_name}/raw_data/${exp_name}_${SLURM_ARRAY_TASK_ID}_${i}/collision.xml -m 1 | wc -l) -eq 0 ]; then
            echo "Removing ${DIR_NAME}/${exp_name}/raw_data/${exp_name}_${SLURM_ARRAY_TASK_ID}_${i}"
            # if del_mode is verbose, only remove fcd_all.xml
            if [ "${del_mode}" = "verbose" ]; then
                rm -f ${DIR_NAME}/${exp_name}/raw_data/${exp_name}_${SLURM_ARRAY_TASK_ID}_${i}/fcd_all.xml
            else
                rm -rf ${DIR_NAME}/${exp_name}/raw_data/${exp_name}_${SLURM_ARRAY_TASK_ID}_${i}
            fi
        fi
    fi
done
