#!/usr/bin/env bash
nohup roslaunch sailboat_launch set_environment_onshore.launch &
sleep 5s
nohup roslaunch sailboat_launch interface_onshore.launch &
sleep 5s
echo "start launch onshore"
rosnode list
