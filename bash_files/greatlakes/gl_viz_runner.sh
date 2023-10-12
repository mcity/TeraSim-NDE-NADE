#!/bin/bash

#SBATCH --job-name="safetest-visualization"
#SBATCH --mail-user=haoweis@umich.edu
#SBATCH --mail-type=BEGIN,END,FAIL
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=7gb
#SBATCH --array=0-99 # how many workers you are using
#SBATCH --time=00-02:00:00 # time duration
#SBATCH --account=henryliu98
#SBATCH --partition=standard
#SBATCH --output=/home/haoweis/viz.log # change to your directory



ulimit -c 0
cd /home/haoweis
source .bashrc
cd /home/haoweis/Safe-Test-TeraSim
module load python/3.10.4
module load ffmpeg
source /venv/bin/activate

# cd /home/hyfrankl
# source .bashrc
# cd /home/hyfrankl/Safe-Test-TeraSim
# module load python/3.10.4
# module load ffmpeg
# source /home/hyfrankl/terasim-venv/bin/activate


DIR_NAME="/scratch/henryliu_root/henryliu98/shared_data/safetest-nade"
check_mode="neg_caused_check"
mode="ITE_calibration_change_2roundabout_out_priority"
USER="haoyang"


FILLED_SLURM_ARRAY_TASK_ID=$(printf "%02d" $SLURM_ARRAY_TASK_ID)
file="split_files/${check_mode}_${FILLED_SLURM_ARRAY_TASK_ID}"

python viz_runner.py --file $file --user ${USER} --mode $mode --exp_repo ${DIR_NAME} --check ${check_mode}
