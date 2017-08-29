#!/bin/bash

echo "start tld_tracker..."

model_file="$(rospack find tld_tracker)/tld_models/better_feature_0829"

roslaunch tld_tracker ros_tld_tracker.launch image_topic:=/camera/image_raw load_model:=true model_import_file:=$model_file

