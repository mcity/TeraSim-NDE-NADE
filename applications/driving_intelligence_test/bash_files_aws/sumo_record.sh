#!/bin/bash
sleep 3
echo "start recording cav context from redis..."

cd /home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test
python3 sumo_record_aws.py
