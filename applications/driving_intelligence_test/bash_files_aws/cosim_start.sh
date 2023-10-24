#!/bin/bash

export path_to_autoware=/home/ubuntu

redis-server & 
bash -c "/home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files_aws/sumo_launch.sh ; exec bash" &
bash -c "/home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files_aws/sumo_record.sh ; exec bash" & 
bash -c "/home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files_aws/autoware_launch.sh ; exec bash" &
bash -c "/home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files_aws/autoware_record.sh ; exec bash" &
bash -c "/home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files_aws/autoware_interrupt.sh ; exec bash" &