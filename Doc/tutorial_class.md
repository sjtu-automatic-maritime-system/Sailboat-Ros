## experiment
0 配置ip

1 复制文件(terminal1)
```
scp ~/sailboat_ws/src/your_function.tar.gz sjtu151@192.168.1.51:/home/sjtu151/sailboat_ws/src/
```

2 远程登入(terminal1)
```
ssh sjtu151@192.168.1.51 
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
./start_launch_onshore_driver.sh
./start_launch_onboat.sh
```

5 启动岸上程序(terminal2)
```
cd ~/sailboat_ws/src/Sailboat-Ros/Sh/onshore
./start_launch_onshore.sh
```
观察通讯和传感器数据是否正常
若出现rviz崩溃现象
```
./rviz.sh
```

6 启动autopilot算法(terminal1)
```
cd ~/sailboat_ws/src/your_function/sh
./your_function.sh
```

## simulation

0 
```
roslaunch sailboat_launch start_tf_tree_onboat.launch
```

1 
```
roslaunch spare_function spare_function_simulation.launch
```

2 
```
cd Sh/simulation
./interface.sh
```
