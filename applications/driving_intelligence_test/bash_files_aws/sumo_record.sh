#!/bin/bash
sleep 3

cd /home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test
while true
do
    # Use redis-cli to get the value of the key. Change mykey to your key.
    value=$(redis-cli get launch_autoware)
    
    if [[ $value -eq 1 ]]
    then
    	python3 sumo_record_aws.py
    fi

    # Wait for a while before checking again.
    sleep 1
done