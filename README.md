# SJTU Sailboat Project


####1. install
```
mkdir -p ~/sailboat_ws/src
cd ~/sailboat_ws/src
git clone https://github.com/hywel1994/Sailboat-Ros.git
cd ~/sailboat_ws/src/Sailboat-Ros/Sh/install
./install_ros.sh
```

####2. catkin build
```$xslt
cd ~/sailboat_ws
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
catkin build
```
####3.tutorial

[class tutorial](https://github.com/hywel1994/Sailboat-Ros/blob/kinetic/Doc/tutorial_class.md)


