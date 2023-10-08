#!/bin/bash

while true
do
    # Use redis-cli to get the value of the key. Change mykey to your key.
    value=$(redis-cli get launch_autoware)
    
    if [ -n "$value" ]
    then
        if [[ $value -eq 0 ]]
        then
            # Get a list of ROS2 process IDs only.
            pids=$(pgrep -f ros2)

            for pid in $pids
            do
                # Send an interrupt signal to each ROS2 process.
                kill -INT $pid
            done
            
            # Explicitly send an interrupt signal to rviz processes
            rviz_pids=$(pgrep -f rviz)
            for pid in $rviz_pids
            do
                kill -INT $pid
            done

	    echo "Stop autoware signal, waiting for 40 seconds..."
            sleep 40
        fi
    fi

    # Wait for a while before checking again.
    sleep 1
done
