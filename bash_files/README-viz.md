### Greatlakes Visualizer
---

#### Files
- `split_file.sh`: generate the split files in split_files in the current directory
```
exp_repo="/scratch/henryliu_root/henryliu0/shared_data/safetest-nade"
mode="ITE_1e-7negligence_1e-2IS_disable_leftright_foll_negligence_ann_arbor_IDM_MOBIL_henryliu0"
user="haoyang"
check_mode="quick_check"
```
- `viz_runner.py`: python script to generate videos 
```
usage: viz_runner.py [-h] [--file FILE] [--user USER] [--mode MODE]
                     [--exp_repo EXP_REPO] [--check CHECK]

optional arguments:
  -h, --help           show this help message and exit
  --file FILE
  --user USER
  --mode MODE
  --exp_repo EXP_REPO
  --check CHECK
```
- `gl_viz_runner.sh`: bash file to call `viz_runner.py`
```
DIR_NAME="/scratch/henryliu_root/henryliu0/shared_data/safetest-nade"
check_mode="quick_check"
mode="ITE_1e-7negligence_1e-2IS_disable_leftright_foll_negligence_ann_arbor_IDM_MOBIL_henryliu0""
USER="haoyang"
```

#### Function checking
- the following bash file can check the total line of the files in `split_files` 
```
#!/bin/bash

x=0

for file in $(ls split_files); do
    x=$((x+$(wc -l split_files/$file | awk '{print $1}')))
done

echo $x
```

#### Usage
- first `bash split_file.sh` to generate split files
- check the name of files in split_files directory by `ls split_files`
- run `sbatch gl_viz_runner.sh` to generate
- check the log file defined in `gl_viz_runner.sh` to see if everything is ok.