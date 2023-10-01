#!/bin/bash
  
# for SLURM_ARRAY_TASK_ID in {1..20}; do
#     FILLED_SLURM_ARRAY_TASK_ID=$(printf "%02d" $SLURM_ARRAY_TASK_ID)
#     echo $FILLED_SLURM_ARRAY_TASK_ID
# done
x=0

for file in $(ls split_files); do
    x=$((x+$(wc -l split_files/$file | awk '{print $1}')))
done

echo $x
