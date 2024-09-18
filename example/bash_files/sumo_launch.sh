#!/bin/bash
sleep 1

export HAS_LIBSUMO=1
export IS_MAGNITUDE_INTERSECTION=100
export IS_MAGNITUDE_ROUNDABOUT=400
export IS_MAGNITUDE_HIGHWAY=400
export AVOID_COLLISION_IS_PROB=0.6

# Generate a hash using md5sum
hash=$(echo -n "$HOSTNAME" | md5sum | awk '{print $1}')
# Convert the hexadecimal hash to a decimal number
unique_number=$(echo "ibase=16; $(echo $hash | tr '[:lower:]' '[:upper:]')" | bc)
# Optionally, if the number is too large, take a subset of it
HOST_ID=${unique_number:0:10}

MODE="isuzu"

DIR_NAME="$HOME/Isuzu/TeraSim-NDE-ITE/output"
LUGURU_DIR="$DIR_NAME/$MODE/aggregated_data/$HOST_ID"

mkdir -p ${DIR_NAME}/${MODE}/raw_data

REDIS_CLI_CMD="redis-cli -h localhost -p 6379"

for i in {1..1000}; do    
    echo "launch autoware"
    $REDIS_CLI_CMD set av_status 1
    echo "set test mode"
    $REDIS_CLI_CMD set test_mode ${MODE}
    echo "set iteration number ${i}"
    $REDIS_CLI_CMD set iteration ${i}
    echo "set host id ${HOST_ID}"
    $REDIS_CLI_CMD set host_id $HOST_ID
    
    # Get the current timestamp
    current_timestamp=$(date +%s)
    
    echo "initializing autoware, waiting for 30 seconds..."
    sleep 30
    
    python3 isuzu_test_highway.py --dir ${DIR_NAME} --name ${MODE} --nth "${HOST_ID}_${current_timestamp}" --aggregateddir ${LUGURU_DIR}

    $REDIS_CLI_CMD set av_status 0

    echo "shuting down autoware, waiting for 30 seconds..."
    
    sleep 15
    
    $REDIS_CLI_CMD flushall
    
    sleep 15
done
