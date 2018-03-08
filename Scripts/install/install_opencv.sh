#!/usr/bin/env bash
sudo apt install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev -y
sudo apt install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev -y
sudo apt install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev liblapacke-dev -y
sudo apt install libxvidcore-dev libx264-dev libatlas-base-dev gfortran ffmpeg -y
sudo apt-get install unzip -y

mkdir -p ~/ct && cd ~/ct
#https://www.tecmint.com/rename-downloaded-file-with-wget-in-linux/
wget https://github.com/opencv/opencv/archive/3.3.1.zip -O opencv.zip
unzip opencv.zip
wget https://github.com/opencv/opencv_contrib/archive/3.3.1.zip -O opencv_contrib.zip
unzip opencv_contrib.zip
#opencv_contrib-3.3.1

cd opencv-3.3.1
mkdir build
cd build

#cmake ..
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.3.1/modules ..

make -j4
sudo make install


