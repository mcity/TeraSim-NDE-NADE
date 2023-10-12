#!/bin/bash

exp_repo="/scratch/henryliu_root/henryliu98/shared_data/safetest-nade"
mode="ITE_calibration_change_2roundabout_out_priority"
user="haoyang"
check_mode="neg_caused_check"
export_stat_dir="${exp_repo}/${mode}/${user}/processed_data"

rm -rf split_files
mkdir -p split_files
chmod 775 split_files

check_file="${export_stat_dir}/${check_mode}_list.txt"
split -n l/100 ${check_file} split_files/${check_mode}_ -d --suffix-length=2
