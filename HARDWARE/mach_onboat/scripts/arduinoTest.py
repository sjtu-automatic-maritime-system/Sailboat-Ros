#!/usr/bin/env python
import rospy
import struct
import binascii
import serial
from mach_onboat.msg import Mach_msg
# from msgdev import

fst = struct.Struct('!4H')
line = ''
motor = 0
rudder = 0
sail = 0

data = 0


def callback(data):
    global motor, rudder, sail
    #print ('start')
    #rospy.loginfo("I heard %f", data.roll)
    motor = int(data.motor*57.3)+90
    rudder = int(data.rudder*57.3)+90
    sail = int(data.sail*57.3)+90

    print motor,rudder,sail

    tmpmain()



def crc16(x):
    poly = 0x8408
    crc = 0x0
    for byte in x:
        crc = crc ^ ord(byte)
        for i in range(8):
            last = (0xFFFF & crc) & 1
            crc = (0xffff & crc) >> 1
            if last == 1:
                crc = crc ^ poly
    return crc & 0xFFFF


def send_data(data):
    header = '\xff\x01'
    tmp = fst.pack(motor,rudder,sail,data)
    crc_code = struct.pack('!H', crc16(tmp))
    # print binascii.hexlify(tmp), type(tmp)
    tmp = header + tmp
    tmp = tmp + crc_code
    print binascii.hexlify(tmp)
    return tmp


def read_data(ser):
    global line
    l = ser.readline()
    print l
    if l == '':
        print 2
        return
    line = line + l
    if line.endswith('\n'):
        print line
        line = ''

def tmpmain():
    global data
    data += 1
    if data > 360:
        data = 0
    Arduino_ser.write(send_data(data))
    print motor, rudder, sail,data
    read_data(Arduino_ser)

class SensorListener:
    def __init__(self,nodeName,topicName):
        self.NodeName = nodeName
        self.TopicName = topicName


    def listener(self):
        rospy.init_node(self.NodeName, anonymous=True)

        #rospy.Subscriber("Ahrs", Ahrs_msg, callback)
        rospy.Subscriber(self.TopicName, Mach_msg, callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()




if __name__ == '__main__':
    port = '/dev/serial/by-id/usb-Arduino_Srl_Arduino_Mega_855353036363516120C1-if00'
    # port = '/dev/ttyACM0'
    Arduino_ser = serial.Serial(port=port, baudrate=115200, bytesize=8,
                                parity = 'N', stopbits=1, timeout=1)

    mach = SensorListener('arduinoTest','mach')
    mach.listener()

    # send_data(20)
