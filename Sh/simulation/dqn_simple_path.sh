#!/usr/bin/env bash
nohup rosrun deep_q_learning simple_path_main.py &
sleep 2
nohup roslaunch sailboat_launch start_tf_tree_onboat.launch &
sleep 2s
nohup roslaunch sailboat_simulation dqn_path_following_simulation.launch &
sleep 2s
nohup rosrun rviz rviz -d rviz/sim.rviz &
sleep 3s
nohup rosrun rqt_gui rqt_gui --perspective-file rqt/sim.perspective &
sleep 3s
echo "start scanning simulation"
rosnode list