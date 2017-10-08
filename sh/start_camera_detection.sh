#!/usr/bin/env bash
nohup roslaunch pointgrey_camera_driver camera.launch &
sleep 2s
model_file="$(rospack find tld_tracker)/tld_models/norway_inside_0906"
#model_file="$(rospack find tld_tracker)/tld_models/better_feature_0829"
#model_file="$(rospack find tld_tracker)/tld_models/norway_sea_0906_1"
nohup roslaunch tld_tracker ros_tld_tracker.launch image_topic:=/camera/image_raw load_model:=true model_import_file:=$model_file &
echo "load model: $model_file"
echo "start camera detection"
#nohup rosbag record --split --duration 5m -j -o ~/BAG/camera /camera/image_raw /tld_tracked_object /tld_fps &
rosnode list
