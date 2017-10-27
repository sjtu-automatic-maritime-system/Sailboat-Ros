#!/usr/bin/env bash
#nohup rosrun sensor_onboat GPS_Talker2.py &
#sleep 2s
nohup rosrun path_planning_astar astar_ros &
sleep 2s
nohup rosrun traj_following traj_following_ros &
sleep 2s
nohup rosbag record --split --duration 5m -j -o ~/BAG/path_planning /wtst_pro /traj_following_out /traj_following_para /planned_path &
rosnode list