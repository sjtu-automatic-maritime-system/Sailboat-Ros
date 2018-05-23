不再使用这个方法

ros by example

sudo apt-get install chrony

On the robot
export ROS_HOSTNAME=sjtu-sailboat.local

On the desktop
export ROS_HOSTNAME=sjtu-OptiPlex.local
export ROS_MASTER_URI=http://sjtu-sailboat.local:11311
sudo ntpdate -b sjty-sailboat.local



now
On the robot
sudo vim /ect/hosts
192.168.1.151 sailboat-desktop

On the desktop
sudo vim /ect/hosts
192.168.1.150 sailboat-robot


