#!/bin/bash

echo "run..."
gnome-terminal --tab --title "roscore" -e "sh -c 'roscore;exec bash'" \
--tab --title "obs_show" -e "sh -c 'sleep 2; python ~/catkin_ws/src/Sailboat-Ros/PATH_PLANNING/python/rviz_plot.py '" \
--tab --title "rviz" -e "sh -c 'sleep 2;rviz -d path_planning.rviz'" \
--tab --title "rqt" -e "sh -c 'sleep 2;rosrun rqt_gui rqt_gui --perspective-file rqt_path_planning.perspective '" \
