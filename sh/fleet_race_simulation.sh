nohup roslaunch sailboat_launch start_tf_tree_onboat.launch &
nohup roslaunch sailboat_simulation fleet_race_simulation.launch &
nohup roslaunch sailboat_launch interface_onshore.launch &
echo "start scanning simulation"
rosnode list