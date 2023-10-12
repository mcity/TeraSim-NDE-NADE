#!/bin/bash

#SBATCH --job-name="terasim safe test"
#SBATCH --mail-user=hyfrankl@umich.edu
#SBATCH --mail-type=BEGIN,END,FAIL
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=7gb
#SBATCH --array=0-99 # how many workers you are using
#SBATCH --time=00-02:00:00 # time duration
#SBATCH --account=henryliu98
#SBATCH --partition=standard
#SBATCH --output=/home/hyfrankl/safe_test.log # change to your directory



ulimit -c 0
cd /home/hyfrankl
source .bashrc
cd /home/hyfrankl/Safe-Test-TeraSim
module load python/3.10.4
source /home/hyfrankl/terasim-venv/bin/activate

DIR_NAME="/scratch/henryliu_root/henryliu98/shared_data/safetest"
export HAS_LIBSUMO=1

mode="nde-check"
mkdir -p ${DIR_NAME}/${mode}
del_mode="verbose"

for i in {1..300}; do
    mkdir -p ${DIR_NAME}/${mode}/${mode}_${SLURM_ARRAY_TASK_ID}_${i}
    python safetest_mcity_main.py --dir ${DIR_NAME} --mode ${mode} --nth ${SLURM_ARRAY_TASK_ID}_${i} > ${DIR_NAME}/${mode}/${mode}_${SLURM_ARRAY_TASK_ID}_${i}/res.txt
    # remove if no collision with del_mode
    if [ $(grep "victim" ${DIR_NAME}/${mode}/${mode}_${SLURM_ARRAY_TASK_ID}_${i}/collision.xml | wc -l) -eq 0 ]; then
        echo "Removing ${DIR_NAME}/${mode}/${mode}_${SLURM_ARRAY_TASK_ID}_${i}"
        # if del_mode is verbose, only remove fcd_all.xml
        if [ "${del_mode}" = "verbose" ]; then
            rm -f ${DIR_NAME}/${mode}/${mode}_${SLURM_ARRAY_TASK_ID}_${i}/fcd_all.xml
        else
            rm -rf ${DIR_NAME}/${mode}/${mode}_${SLURM_ARRAY_TASK_ID}_${i}
        fi
    fi
done
