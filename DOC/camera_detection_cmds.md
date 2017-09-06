
camera:
roslaunch pointgrey_camera_driver camera.launch

tld:
roslaunch tld_tracker ros_tld_tracker.launch image_topic:=/camera/image_raw load_model:=false
roslaunch tld_tracker ros_tld_gui.launch image_topic:=/camera/image_raw


rosbag:
rosbag record --split --duration 5m -j /camera/image_raw /tld_tracked_object /tld_fps

