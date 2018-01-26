http://www.cnblogs.com/CZM-/p/6113475.html

Bus 001 Device 012: ID 067b:2303 Prolific Technology, Inc. PL2303 Serial Port

lrwxrwxrwx 1 root root 0  4月 17 21:13 ttyUSB0 -> ../../devices/pci0000:00/0000:00:14.0/usb1/1-10/1-10:1.0/ttyUSB0/tty/ttyUSB0
lrwxrwxrwx 1 root root 0  4月 17 21:14 ttyUSB1 -> ../../devices/pci0000:00/0000:00:14.0/usb1/1-9/1-9:1.0/ttyUSB1/tty/ttyUSB1

DEVPATH=="/sys/devices/pci0000:00/0000:00:14.0/usb1/1-9*"
/devices/pci0000:00/0000:00:14.0/usb1/1-9/1-9:1.0

USB端口号绑定
/etc/udev/rules.d/ahrs.rules
KERNEL=="ttyUSB*",DEVPATH=="/devices/pci0000:00/0000:00:14.0/usb1/1-9*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0777", SYMLINK+="ahrs"

