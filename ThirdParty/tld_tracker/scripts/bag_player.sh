#!/bin/bash

#TODO
# 1. replace the bagfiles path with your own
#--tab --title "rviz" -e "sh -c 'sleep 2;rviz -d bag_visulization.rviz'" \

#bag_file="/media/jianyun/MyPassport/KITTI_DATA/RawData/Data/BAG/2011_09_26/kitti_2011_09_26_drive_0015_synced.bag"
bag_file="/home/jianyun/BAG/bag_0824/bag_0824_water/camera/test1.bag"

echo "load ros user settings..."
source ~/catkin_ws/devel/setup.bash

echo "run..."
gnome-terminal --tab --title "roscore" -e "sh -c 'roscore;exec bash'" \
--tab --title "use_sim_time" -e "sh -c 'sleep 1; rosparam set use_sim_time true'" \
--tab --title "bagfiles" -e "sh -c 'sleep 2; rosbag info ${bag_file}; rosbag play ${bag_file} --clock -l -r 1'" \


