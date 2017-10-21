source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash

cd ~/catkin_ws/src

catkin_create_pkg beginner_tutorials std_msgs rospy roscpp

chmod +x talker.py

catkin_make -DCATKIN_WHITELIST_PACKAGES="sailboat_message"
catkin_make --pkg sailboat_message



