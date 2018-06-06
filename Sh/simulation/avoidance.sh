#!/usr/bin/env bash
rosnode kill -a
nohup roslaunch sailboat_launch start_tf_tree_onboat.launch &
sleep 3s
nohup roslaunch avoidance_v2 avoidance_v2.launch &
sleep 3s
nohup rosrun rviz rviz -d rviz/sim_2.rviz &
sleep 0.5s
nohup rosrun rqt_gui rqt_gui --perspective-file rqt/sim.perspective &
sleep 3s
echo "start scanning simulation"
rosnode list