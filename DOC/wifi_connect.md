板子：
扫描wifi
sudo nmcli d wifi list
连接wifi
nmcli d wifi connect <WIFISSID> password <password> iface wlan0
连接已保存wifi
sudo nmcli c up id xujianyun
关闭wifi
sudo nmcli d disconnect iface wlan0

电脑
扫描局域网ip
nmap -sP 172.20.10.0/24

将数据拷贝出来
scp sjtu-sailboat@192.168.1.151:/home/sjtu-sailboat/rosbag/fleet_race_* /home/hywel/fleet_race_0904/

