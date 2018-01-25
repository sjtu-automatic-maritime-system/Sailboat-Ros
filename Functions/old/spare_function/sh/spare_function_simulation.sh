#!/usr/bin/env bash
nohup roslaunch sailboat_launch start_tf_tree_onboat.launch &
sleep 5s
nohup roslaunch spare_function spare_function_simulation.launch &
sleep 5s
nohup roslaunch sailboat_launch interface_onshore.launch &
sleep 5s
echo "start scanning simulation"
rosnode list