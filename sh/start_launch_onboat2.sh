#!/usr/bin/env bash
nohup roslaunch sailboat_launch start_arduino.launch &
sleep 5s
echo "start launch onboat"
rosnode list
