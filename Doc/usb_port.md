http://www.cnblogs.com/CZM-/p/6113475.html

Bus 001 Device 012: ID 067b:2303 Prolific Technology, Inc. PL2303 Serial Port

lrwxrwxrwx 1 root root 0  4月 17 21:13 ttyUSB0 -> ../../devices/pci0000:00/0000:00:14.0/usb1/1-10/1-10:1.0/ttyUSB0/tty/ttyUSB0
lrwxrwxrwx 1 root root 0  4月 17 21:14 ttyUSB1 -> ../../devices/pci0000:00/0000:00:14.0/usb1/1-9/1-9:1.0/ttyUSB1/tty/ttyUSB1

DEVPATH=="/sys/devices/pci0000:00/0000:00:14.0/usb1/1-9*"
/devices/pci0000:00/0000:00:14.0/usb1/1-9/1-9:1.0

restart
sudo /etc/init.d/udev restart

USB端口号绑定
/etc/udev/rules.d/ahrs.rules
KERNEL=="ttyUSB*",DEVPATH=="/devices/pci0000:00/0000:00:14.0/usb1/1-9*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0777", SYMLINK+="ahrs"


udevadm info --attribute-walk --name=/dev/ttyUSB0

Udevadm info starts with the device specified by the devpath and then
walks up the chain of parent devices. It prints for every device
found, all possible attributes in the udev rules key format.
A rule to match, can be composed by the attributes of the device
and the attributes from one single parent device.

  looking at device '/devices/pci0000:00/0000:00:14.0/usb1/1-3/1-3.1/1-3.1:1.0/ttyUSB0/tty/ttyUSB0':
    KERNEL=="ttyUSB0"
    SUBSYSTEM=="tty"
    DRIVER==""

  looking at parent device '/devices/pci0000:00/0000:00:14.0/usb1/1-3/1-3.1/1-3.1:1.0/ttyUSB0':
    KERNELS=="ttyUSB0"
    SUBSYSTEMS=="usb-serial"
    DRIVERS=="ch341-uart"
    ATTRS{port_number}=="0"

  looking at parent device '/devices/pci0000:00/0000:00:14.0/usb1/1-3/1-3.1/1-3.1:1.0':
    KERNELS=="1-3.1:1.0"
    SUBSYSTEMS=="usb"
    DRIVERS=="ch341"
    ATTRS{authorized}=="1"
    ATTRS{bAlternateSetting}==" 0"
    ATTRS{bInterfaceClass}=="ff"
    ATTRS{bInterfaceNumber}=="00"
    ATTRS{bInterfaceProtocol}=="02"
    ATTRS{bInterfaceSubClass}=="01"
    ATTRS{bNumEndpoints}=="03"
    ATTRS{supports_autosuspend}=="1"



