nohup roslaunch sailboat_launch start_tf_tree_onboat.launch &
nohup roslaunch sailboat_simulation station_keeping_simulation.launch &
nohup roslaunch sailboat_launch interface_onshore.launch &
echo "start fleet_race simulation"
rosnode list