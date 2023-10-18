#!/bin/bash

gnome-terminal --tab -- redis-server
gnome-terminal --tab -- bash -c "/home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files_aws/sumo_launch.sh ; exec bash"
gnome-terminal --tab -- bash -c "/home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files_aws/sumo_record.sh ; exec bash"
gnome-terminal --tab -- bash -c "/home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files_aws/autoware_launch.sh ; exec bash"
gnome-terminal --tab -- bash -c "/home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files_aws/autoware_record.sh ; exec bash"
gnome-terminal --tab -- bash -c "/home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files_aws/autoware_interrupt.sh ; exec bash"
