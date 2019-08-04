#!/usr/bin/env bash
nohup roslaunch sailboat_launch set_environment_onboat.launch &
sleep 2s
nohup roslaunch sailboat_launch start_tf_tree_onboat.launch &
sleep 5s
nohup rosrun sensor_process kalman4.py &
sleep 5s
echo "start launch onboat"
rosnode list
