#!/usr/bin/env bash
nohup rosbag record --split --duration 5m -j -o /home/sjtu/rosbag -a &
#/ahrs /wtst /mach /base/mach /arduino /sensor /sensor2 /sensor_kalman_msg &
#nohup rosbag record --split --duration 5m -j -o ~/rosbag/camera /camera/image_raw &
