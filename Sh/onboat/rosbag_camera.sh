#!/usr/bin/env bash
nohup rosbag record --split --duration 5m -j -o ~/rosbag/camera /camera/image_raw &