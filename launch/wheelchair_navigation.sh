#!/bin/bash

# Set permissions for the Kangaroo motor controller
sudo chmod 777 /dev/ttyACM0

SESSION_NAME="crowd_surfer"
WINDOW_NAME="crowd_surfer_ros"
CONDA_ENV="drlvo"

tmux kill-session -t $SESSION_NAME 2>/dev/null

# Start a new tmux session and window
sleep 5

# Launch wheelchair hardware driver
tmux new-session -d -s $SESSION_NAME -n "kangaroo motor driver"
tmux send-keys -t $SESSION_NAME:0 ". devel/setup.bash && roslaunch kangaroo_driver kangaroo_driver.launch" C-m

# Launch 3D LiDAR
tmux new-window -d -s $SESSION_NAME -n "3D Lidar"
tmux send-keys -t $SESSION_NAME:1 ". devel/setup.bash && roslaunch wheelchair_laser wheelchair_msg_MID360.launch" C-m

# launch the realsense camera
tmux new-window -d -s $SESSION_NAME -n "Realsense Camera"
tmux send-keys -t $SESSION_NAME:2 ". devel/setup.bash && roslaunch realsense2_camera rs_camera.launch" C-m

# Launch navigation using fastlio 
tmux new-window -d -s $SESSION_NAME -n "Wheelchair fastlio"
tmux send-keys -t $SESSION_NAME:3 ". devel/setup.bash && roslaunch crowdsurfer_ros wheelchair_nav_fastlio.launch" C-m

# Launch LV-DOT
tmux new-window -d -s $SESSION_NAME -n "LV-DOT"
tmux send-keys -t $SESSION_NAME:4 ". devel/setup.bash && roslaunch onboard_detector run_detector_wheelchair.launch" C-m

# ROS interface runfile 
tmux new-window -d -s $SESSION_NAME -n "ROS interface"
tmux send-keys -t $SESSION_NAME:5 "conda activate drvlo && . devel/setup.bash && rosrun crowdsurfer_ros ros_interface.py " C-m

tmux attach -t $SESSION_NAME