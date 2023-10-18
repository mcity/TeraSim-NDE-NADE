#!/bin/bash

redis-server &
bash /home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files_aws/sumo_launch.sh &
bash /home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files_aws/sumo_record.sh &
bash /home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files_aws/autoware_launch.sh &
bash /home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files_aws/autoware_record.sh &
bash /home/ubuntu/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files_aws/autoware_interrupt.sh &