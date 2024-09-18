#!/bin/bash

gnome-terminal --tab -- redis-server
gnome-terminal --tab -- redis-commander

REDIS_CLI_CMD="redis-cli -h localhost -p 6379"

$REDIS_CLI_CMD flushall

gnome-terminal --tab -- bash -c "$HOME/Isuzu/TeraSim-NDE-ITE/example/bash_files/sumo_launch.sh ; exec bash"
gnome-terminal --tab -- bash -c "$HOME/Isuzu/TeraSim-NDE-ITE/example/bash_files/autoware_stack_launch.sh ; exec bash"
gnome-terminal --tab -- bash -c "$HOME/Isuzu/TeraSim-NDE-ITE/example/bash_files/autoware_cosim_launch.sh ; exec bash"
gnome-terminal --tab -- bash -c "$HOME/Isuzu/TeraSim-NDE-ITE/example/bash_files/autoware_interrupt.sh ; exec bash"
