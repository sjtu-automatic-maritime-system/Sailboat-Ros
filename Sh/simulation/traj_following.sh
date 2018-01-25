#!/usr/bin/env bash
nohup roslaunch sailboat_launch start_tf_tree_onboat.launch &
sleep 5s
nohup roslaunch sailboat_simulation traj_following_simulation.launch &
sleep 5s
#nohup roslaunch sailboat_launch interface_onshore.launch &
#sleep 5s
echo "start traj_following simulation"
rosnode list