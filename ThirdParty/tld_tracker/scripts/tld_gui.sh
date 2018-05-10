#!/bin/bash

echo "start tld_gui..."

roslaunch tld_tracker ros_tld_gui.launch image_topic:=/camera/image_raw

