#!/usr/bin/env bash
nohup roscore &
sleep 2s
nohup rosrun actuator_onboat arduino_comm.py &
sleep 5s
nohup rosrun sensor_onboat Ahrs_Talker.py &
sleep 5s
nohup rosrun sensor_onboat WeatherStation_Talker.py &
sleep 5s
nohup rosrun actuator_onboat dxl_driver_new2.py &
sleep 5s
nohup rosrun actuator_onboat base_control &
sleep 3s
#nohup roslaunch actuator_onboat dynamixel_position_control.launch &
#sleep 5s
echo "start launch onboat driver"
rosnode list
