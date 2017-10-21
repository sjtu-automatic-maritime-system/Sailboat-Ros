#!/bin/bash

echo "run simulation..."
gnome-terminal --tab --title "roscore" -e "sh -c 'roscore;exec bash'" \
--tab --title "wind_pub" -e "sh -c 'sleep 1; rosrun environment_simulation wind_simulation_pub'" \
--tab --title "sailboat_model" -e "sh -c 'sleep 1; rosrun sailboat_simulation sailboat_simulation_ver1'" \
--tab --title "path_planning" -e "sh -c 'sleep 1; rosrun path_planning_astar astar_simulation_ros'" \
--tab --title "traj_following" -e "sh -c 'sleep 1; rosrun traj_following traj_following_ros'" \
--tab --title "obs_show" -e "sh -c 'sleep 2; python ~/catkin_ws/src/Sailboat-Ros/PATH_PLANNING/python/rviz_plot_simulation.py '" \
--tab --title "rviz" -e "sh -c 'sleep 2;rviz -d path_planning.rviz'" \
