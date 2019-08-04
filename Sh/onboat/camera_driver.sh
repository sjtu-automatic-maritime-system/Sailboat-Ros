#!/usr/bin/env bash:
nohup roslaunch pointgrey_camera_driver camera.launch &
sleep 2s
rosnode list
