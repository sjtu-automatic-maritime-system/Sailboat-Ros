#!/usr/bin/env bash
nohup rosbag record --split --duration 60m -j -o /home/sjtu151/rosbag /ahrs /wtst /mach /base/mach /arduino /sensor_kalman_msg /station_keeping_out /fleet_race_out /scanning_out &
#nohup rosbag record --split --duration 5m -j -o ~/rosbag/camera /camera/image_raw &
