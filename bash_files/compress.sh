#!/bin/bash

#SBATCH --job-name="mtlsp safe test"
#SBATCH --mail-user=hyfrankl@umich.edu
#SBATCH --mail-type=BEGIN,END,FAIL
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=7gb
#SBATCH --array=0 # how many workers you are using
#SBATCH --time=00-02:00:00 # time duration
#SBATCH --account=henryliu0
#SBATCH --partition=standard
#SBATCH --output=/home/hyfrankl/safe_test.log # change to your directory



ulimit -c 0

MAIN_PATH="/scratch/henryliu_root/henryliu0/shared_data/"
DIR_NAME="safetest-nade"

cd $MAIN_PATH

mkdir -p compressed_data
mkdir -p blank

cd $DIR_NAME

for file in $(ls); do
    echo $file
    if [ -d $file ]; then
        zip -r $MAIN_PATH/compressed_data/$file.zip $file
        # rsync --delete-before --force -r $MAIN_PATH/blank $file
    fi
done
