1 复制文件(terminal1)
```
scp ~/catkin_ws/src/your_function sjtu-sailboat@192.168.1.151:/home/sjtu-sailboat/catkin_ws/src/

```

1 远程登入(terminal1)
```
ssh sjtu-sailboat@192.168.1.151 
password sjtu
```
2 编译(terminal1)
```
cd ~/catkin_ws/
catkin make
```

3 启动船上程序(terminal1)
```
cd ~/catkin_ws/src/Sailboat-Ros/sh
./start_launch_onboat1.sh
./start_launch_onboat2.sh
```

4 启动岸上程序(terminal2)
```
cd ~/catkin_ws/src/Sailboat-Ros/sh
start_launch_shore.sh
若出现rviz崩溃现象
roslaunch sailboat_launch interface_onshore.launch
观察通讯和传感器数据是否正常
```

5 启动autopilot算法(terminal1)
```
cd ~/catkin_ws/src/your_function/sh
./your_function.sh
```
