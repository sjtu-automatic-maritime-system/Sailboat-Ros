#!/bin/bash

echo "run..."
gnome-terminal --tab --title "roscore" -e "sh -c 'roscore;exec bash'" \
--tab --title "obs_plot" -e "sh -c 'sleep 1; python ~/catkin_ws/src/Sailboat-Ros/PATH_PLANNING/python/rviz_plot.py'" \
--tab --title "rviz" -e "sh -c 'sleep 2;rviz -d path_planning.rviz'" \
