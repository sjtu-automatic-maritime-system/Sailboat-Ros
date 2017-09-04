nohup roslaunch pointgrey_camera_driver camera.launch &
sleep 2s
model_file="$(rospack find tld_tracker)/tld_models/better_feature_0829"
nohup roslaunch tld_tracker ros_tld_tracker.launch image_topic:=/camera/image_raw load_model:=true model_import_file:=$model_file &
echo "load model: $model_file"
echo "start camera detection"
rosnode list
