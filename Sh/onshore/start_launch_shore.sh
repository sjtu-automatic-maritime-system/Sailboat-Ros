#!/usr/bin/env bash
nohup roslaunch sailboat_launch set_environment_onshore.launch &
sleep 5s
nohup rosrun rviz rviz -d rviz/sim.rviz &
sleep 3s
nohup rosrun rqt_gui rqt_gui --perspective-file rqt/sim.perspective &
sleep 3s
echo "start launch onshore"
rosnode list
