#!/bin/bash

echo "load ros user settings..."

echo "run..."
gnome-terminal --tab --title "roscore" -e "sh -c 'roscore;exec bash'" \

--tab --title "sensor" -e "sh -c 'sleep 2;roslaunch didi_challenge_ros display_rviz.launch'" \

  --tab -T zgw -e "bash -ic '$NODESTART 1 $(rospack find sailboat_launch)/launch start_sensor_onboat.launch'" \
  --tab -T vm -e "bash -ic '$NODESTART 1 $(rospack find sailboat_launch)/launch start_tf_tree_onboat.launch'" \

