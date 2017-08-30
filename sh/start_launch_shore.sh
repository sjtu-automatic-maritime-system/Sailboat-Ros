nohup roslaunch sailboat_launch set_environment_onshore.launch &
sleep 2s
nohup roslaunch sailboat_launch interface_onshore.launch &
sleep 2s
echo "start launch onshore"
rosnode list
