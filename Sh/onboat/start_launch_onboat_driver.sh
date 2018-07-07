#!/usr/bin/env bash

nohup rosrun mach_onboat arduino_comm.py &
sleep 5s
nohup rosrun sensor_onboat Ahrs_Talker.py &
sleep 5s
nohup rosrun sensor_onboat WeatherStation_Talker.py &
sleep 5s
nohup rosrun mach_onboat dxl_driver.py &
sleep 5s
nohup rosrun mach_onboat base_control &
sleep 3s
#nohup roslaunch mach_onboat dynamixel_position_control.launch &
#sleep 5s
echo "start launch onboat driver"
rosnode list
