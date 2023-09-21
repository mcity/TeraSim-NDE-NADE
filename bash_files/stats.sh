#!/bin/bash
module load python/3.10.4
source /home/hyfrankl/terasim-venv/bin/activate

# calculate the distribution of collisions
DIR_NAME="/scratch/henryliu_root/henryliu98/hyfrankl/terasim_output"
mode="nde-check"
echo "Collisions in ${mode}: $(grep "victim" ${DIR_NAME}/${mode}/*/collision.xml | wc -l) / $(ls ${DIR_NAME}/${mode} | wc -l)"
echo "Lane change" in ${mode}: $(grep "laneChange" ${DIR_NAME}/${mode}/*/run.log | wc -l)
grep -oP '(?<=lane=\")[^,]*(?<=\")' /scratch/henryliu_root/henryliu98/hyfrankl/terasim_output/ite-new/*/collision.xml | awk -F "[:\"/ ]" '{print $8, $10, $11}' | sort > record.txt
awk -F " " '{print $2}' record.txt | sort | uniq -c | sort -nr
grep -oP '(?<=lane=\")[^,]*(?<=\")' ${DIR_NAME}/${mode}/*/collision.xml | awk -F "[:\" ]" '!seen[$2,$3]++ {print $1, $2, $3}'

echo "Analyze:"
python stats.py --path ${DIR_NAME}/${mode}
