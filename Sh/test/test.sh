#!/usr/bin/env bash
#sudo chmod +777 /dev/ttyACM0
#sudo chmod +777 /dev/ahrs
#sudo chmod +777 /dev/dynamixel
#sudo chmod +777 /dev/wtst
nohup roslaunch sailboat_launch set_environment_onboat.launch &
sleep 2s
nohup roslaunch sailboat_launch start_drivers.launch &
sleep 10s
nohup roslaunch actuator_onboat dynamixel_position_control.launch &
sleep 5s
