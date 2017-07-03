nohup roslaunch sailboat_launch start_tf_tree_onboat.launch &
nohup roslaunch sailboat_launch start_sensor_onboat.launch &
nohup roslaunch sailboat_launch autopilot_onboat.launch &
nohup roslaunch sailboat_launch rosbag_onboat.launch &
echo "start tf_tree"
