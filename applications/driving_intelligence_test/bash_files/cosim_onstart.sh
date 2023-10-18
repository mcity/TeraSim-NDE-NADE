#!/bin/bash

gnome-terminal --tab -- redis-server
gnome-terminal --tab -- bash -c "/home/zhijie/terasim/Terasim-NDE-ITE/bash_files/sumo_launch.sh ; exec bash"
gnome-terminal --tab -- bash -c "/home/zhijie/terasim/Terasim-NDE-ITE/bash_files/sumo_record.sh ; exec bash"
gnome-terminal --tab -- bash -c "/home/zhijie/terasim/Terasim-NDE-ITE/bash_files/autoware_launch.sh ; exec bash"
gnome-terminal --tab -- bash -c "/home/zhijie/terasim/Terasim-NDE-ITE/bash_files/autoware_record.sh ; exec bash"
gnome-terminal --tab -- bash -c "/home/zhijie/terasim/Terasim-NDE-ITE/bash_files/autoware_interrupt.sh ; exec bash"
