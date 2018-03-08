# SJTU Sailboat Project


####1. rospackage 
multimaster_fkie
pointgrey_camera_driver
camera_umd

####2. catkin build

```$xslt
$ sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools
```

```$xslt
mkdir ~/sailboat_ws/src
cd ~/sailboat_ws
catkin init
cd src
git clone https://github.com/hywel1994/Sailboat-Ros.git
cd ..
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
catkin build
```
####3.tutorial

[class tutorial](https://github.com/hywel1994/Sailboat-Ros/blob/kinetic/Doc/tutorial_class.md)


