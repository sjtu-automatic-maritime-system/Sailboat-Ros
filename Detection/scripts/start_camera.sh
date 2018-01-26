#!/bin/bash

echo "start camera..."
#source catkin_ws/devel/setup.bash

echo "run..."
gnome-terminal --tab --title "roscore" -e "sh -c 'roscore;exec bash'" \
--tab --title "driver" -e "sh -c 'sleep 1; roslaunch pointgrey_camera_driver camera.launch'" \
--tab --title "proc" -e "sh -c 'sleep 2;ROS_NAMESPACE=camera rosrun image_proc image_proc'" \
--tab --title "viewer" -e "sh -c 'sleep 3;rosrun image_view image_view image:=camera/image_raw'"\

