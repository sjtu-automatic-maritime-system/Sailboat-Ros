nohup roslaunch sailboat_launch set_environment_onboat.launch &
sleep 5s
nohup roslaunch sailboat_launch start_tf_tree_onboat.launch &
sleep 5s
nohup roslaunch sailboat_launch start_sensor_onboat.launch &
sleep 5s
echo "start launch onboat"
rosnode list
