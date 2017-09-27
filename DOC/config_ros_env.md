#source /opt/ros/indigo/setup.bash
#export LD_LIBRARY_PATH=/opt/ros/indigo/lib:/opt/ros/indigo/lib/i386-linux-gnu:/usr/local/lib/i386-linux-gnu

rostopic echo -b file.bag -p /topic > data.txt
rosrun rqt_plot rqt_plot /WTST
rosrun rosbag record /WTST

source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash

cd ~/catkin_ws/src
#catkin_create_pkg beginner_tutorials std_msgs rospy roscpp

#chmod +x talker.py

catkin_make -DCATKIN_WHITELIST_PACKAGES="sailboat_message"
catkin_make --pkg sailboat_message



