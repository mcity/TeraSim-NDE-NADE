#!/bin/bash
sleep 3

echo "Monitoring terasim status for autoware interrupt"

REDIS_CLI_CMD="redis-cli -h localhost -p 6379"

while true
do
    # Use redis-cli to get the value of the key. Change mykey to your key.
    value=$($REDIS_CLI_CMD get av_status)
    
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

	    echo "send autoware interrupt signal"
            sleep 40
        fi
    fi

    # Wait for a while before checking again.
    sleep 1
done
