#!/bin/bash

## Bash script for setting up a ROS/Gazebo development environment on Ubuntu LTS (16.04). 
## It installs the common dependencies for all targets (including Qt Creator) and the ROS Kinetic/Gazebo 7 (the default).
##
## Installs:
## - Common dependencies libraries and tools
## - ROS Kinetic (including Gazebo7)

# Common dependencies
echo "Installing common dependencies"
sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt-get install git vim -y 
#sudo apt-get zip qtcreator -y
# Required python packages
sudo apt-get install python-argparse python-empy python-toml python-numpy python-dev python-pip -y
sudo -H pip install --upgrade pip
sudo -H pip install pandas

# ROS Kinetic/Gazebo (ROS Kinetic includes Gazebo7 by default)
## Gazebo simulator dependencies
sudo apt-get install libeigen3-dev libopencv-dev -y

## ROS Gazebo: http://wiki.ros.org/kinetic/Installation/Ubuntu
## Setup keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
## For keyserver connection problems substitute hkp://pgp.mit.edu:80 or hkp://keyserver.ubuntu.com:80 above.
sudo apt-get update
## Get ROS/Gazebo
sudo apt-get install ros-kinetic-desktop-full -y
## Initialize rosdep
sudo rosdep init
rosdep update
## Setup environment variables
rossource="source /opt/ros/kinetic/setup.bash"
if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in .bashrc;
else echo "$rossource" >> ~/.bashrc; fi
eval $rossource
## Get rosinstall

sudo apt-get install python-rosinstall -y

sudo apt-get install ros-kinetic-multimaster-fkie ros-kinetic-camera-umd ros-kinetic-pointgrey-camera-driver -y
sudo apt-get install ros-kinetic-velodyne-simulator ros-kinetic-hector-gazebo-plugins-y
## Create catkin workspace
# careful no space here

## Install dependencies
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential -y
sudo apt-get install python-catkin-tools -y
#sudo apt install ros-kinetic-mavros ros-kinetic-mavros-extras -y
sudo apt install ros-kinetic-control-toolbox -y


catkin_ws=sailboat_ws
mkdir -p ~/$catkin_ws/src
## Initialise wstool
wstool init ~/$catkin_ws/src



