#!/usr/bin/env bash
nohup rosrun deep_q_learning simple_path_main.py &
sleep 2
nohup roslaunch sailboat_launch start_tf_tree_onboat.launch &
sleep 2s
nohup roslaunch sailboat_simulation dqn_path_following_simulation.launch &
sleep 2s
nohup roslaunch sailboat_launch interface_onshore.launch &
sleep 2s
echo "start scanning simulation"
rosnode list