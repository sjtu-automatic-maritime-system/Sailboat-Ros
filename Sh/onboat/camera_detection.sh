#!/usr/bin/env bash
nohup roslaunch pointgrey_camera_driver camera.launch &
sleep 2s
nohup rosrun object_detection_ros object_detection &
sleep 2s
nohup rosbag record --split --duration 5m -j -o ~/rosbag/camera /camera/image_raw /object/pose /tld_tracked_object &
sleep 2s
rosnode list
