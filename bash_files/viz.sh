#!/bin/bash

module load ffmpeg
module load python/3.10.4
source /home/hyfrankl/mtlsp-venv/bin/activate

DIR_NAME="tfl_samples"
mode="ite"

for file in $(ls ${DIR_NAME}); do
     echo ${DIR_NAME}/${file}
     python visualize.py --path ${DIR_NAME}/${file} --file ${file}  --mode ${mode}
done

