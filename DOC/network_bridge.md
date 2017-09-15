1. install bridge-utils
```
apt get install bridge-utils
```
2.打开网口转发
```
echo "1" > /proc/sys/net/ipv4/ip_forward
```
3.在开机启动文件rc.local（/etc/rc.local）中写入（注意在exit 0之前）
```
ifconfig eth0 0.0.0.0 up
ifconfig eth1 0.0.0.0 up
brctl addbr br0
brctl addif br0 eth0
brctl addif br0 eth1
ifconfig br0 192.168.1.151 netmask 255.255.255.0
broadcast 192.168.1.255 up route add default gw 192.168.1.1
```