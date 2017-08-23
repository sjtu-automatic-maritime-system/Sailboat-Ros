nohup roslaunch sailboat_launch set_environment_onboat.launch &
nohup roslaunch sailboat_launch start_tf_tree_onboat.launch &
nohup roslaunch sailboat_launch start_sensor_onboat.launch &
echo "start launch onboat"
rosnode list
