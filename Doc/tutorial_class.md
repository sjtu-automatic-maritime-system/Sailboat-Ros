0 配置ip


1 复制文件(terminal1)
```
scp ~/sailboat_ws/src/your_function.tar.gz sjtu-sailboat@192.168.1.151:/home/sjtu-sailboat/sailboat_ws/src/
```

2 远程登入(terminal1)
```
ssh sjtu-sailboat@192.168.1.151 
password sjtu
```

3 编译(terminal1)
```
cd ~sailboat_ws/src/
tar -zxvf your_function.tar.gz
cd ~/sailboat_ws/
catkin build
```

4 启动船上程序(terminal1)
```
cd ~/sailboat_ws/src/Sailboat-Ros/Sh/onboat
./start_launch_onboat.sh
```

5 启动岸上程序(terminal2)
```
cd ~/sailboat_ws/src/Sailboat-Ros/Sh/onshore
start_launch_onshore.sh
```
若出现rviz崩溃现象
```
roslaunch sailboat_launch interface_onshore.launch
```
观察通讯和传感器数据是否正常

6 启动autopilot算法(terminal1)
```
cd ~/sailboat_ws/src/your_function/sh
./your_function.sh
```
