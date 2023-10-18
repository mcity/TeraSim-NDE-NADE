#!/bin/bash

redis-server &
bash /home/zhijie/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files/sumo_launch.sh &
bash /home/zhijie/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files/sumo_record.sh &
bash /home/zhijie/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files/autoware_launch.sh &
bash /home/zhijie/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files/autoware_record.sh &
bash /home/zhijie/terasim/TeraSim-NDE-ITE/applications/driving_intelligence_test/bash_files/autoware_interrupt.sh &
