experiment_name = "safetest_ite"
DIR_NAME="./output"

for i in {1..300}; do
    mkdir -p ${DIR_NAME}/${experiment_name}/raw_data/${experiment_name}_${i}
    python safetest_mcity_main.py --dir ${DIR_NAME} --mode ${experiment_name}