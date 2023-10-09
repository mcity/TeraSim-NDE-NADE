#!/bin/bash

# gnome-terminal -x bash -c "./bash_files/cosim_autoware_record.sh; bash"

# Create a new tmux session in detached mode
tmux new-session -d -s my_session 

# Split the window into 2 vertical panes
tmux split-window -h

# Split the each pane horizontally to get a 2x2 pane
tmux select-pane -t 0
tmux split-window -v

tmux select-pane -t 2
tmux split-window -v

# By here, you should have a 2x2 grid of tmux panes in your window
# Now you need to run the commands in each pane

# Select pane 0 and run script a
tmux send-keys -t 0 'redis-server' C-m

# Wait for redis server to start
sleep 0.5

# Select pane 1 and run script b
tmux send-keys -t 1 './bash_files/cosim_sumo.sh' C-m 

# Select pane 2 and run script c
tmux send-keys -t 2 './bash_files/cosim_autoware_launch.sh' C-m 

# Select pane 3 and run script d
tmux send-keys -t 3 './bash_files/cosim_autoware_interrupt.sh' C-m 

# Attach the tmux session 
tmux attach-session -t my_session
