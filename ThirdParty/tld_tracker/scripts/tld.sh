#!/bin/bash

#TODO
# 1. replace the bagfiles path with your own
#--tab --title "rviz" -e "sh -c 'sleep 2;rviz -d bag_visulization.rviz'" \

bag_file="/home/jianyun/BAG/bag_0824/bag_0824_water/camera/test2.bag"

echo "load ros user settings..."
source ~/catkin_ws/devel/setup.bash



'roslaunch tld_tracker ros_tld_tracker.launch image_topic:=/camera/image_raw load_model:=true'
'roslaunch tld_tracker ros_tld_gui.launch image_topic:=/camera/image_raw'


'roslaunch tld_tracker ros_tld_tracker.launch image_topic:=/camera/image_raw/compressed'
'roslaunch tld_tracker ros_tld_gui.launch image_topic:=/camera/image_raw/compressed'


'roslaunch tld_tracker ros_tld_gui.launch image_topic:=/kitti/camera_color_left/image_raw'
'roslaunch tld_tracker ros_tld_tracker.launch image_topic:=/kitti/camera_color_left/image_raw'

