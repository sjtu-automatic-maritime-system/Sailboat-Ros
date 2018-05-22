#!/usr/bin/env bash
sudo chmod +777 /dev/ttyACM0
sudo chmod +777 /dev/ahrs
sudo chmod +777 /dev/dynamixel
nohup roslaunch sailboat_launch set_environment_onboat.launch &
sleep 2s
nohup roslaunch sensor_onboat start_sensor.launch &
sleep 10s
nohup roslaunch mach_onboat dynamixel_position_control.launch &
sleep 5s
