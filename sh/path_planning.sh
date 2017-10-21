#!/usr/bin/env bash
nohup rosrun sensor_onboat GPS_Talker2.py &
nohup rosrun path_planning_astar astar_ros &
nohup rosrun traj_following traj_following_ros &
echo "load model: $model_file"
echo "start camera detection"
nohup rosbag record --split --duration 5m -j -o ~/BAG/ /wtst /gps_2 /traj_following_out /planned_path &
rosnode list