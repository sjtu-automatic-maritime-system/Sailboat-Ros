#!/usr/bin/env python
import rospy
from sailboat_message.msg import Mach_msg
from sailboat_message.msg import Arduino_msg
import struct
import binascii
import serial
import time
# from msgdev import

arduino_port = '/dev/serial/by-id/usb-Arduino_Srl_Arduino_Mega_75533353038351615291-if00'
#arduino_port = '/dev/ttyACM0'
motor = 50
rudder = 90
sail = 90
pcCtrl = 0

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

class Arduino():
    def __init__(self):

        self.ser_open_flag = self.ser_open()
        self.DataShow_count = 0
        self.header = chr(0x4f) + chr(0x5e)

        self.fst = struct.Struct('!4H')
        self.recvDataBytes = 18
        self.recvDataFst = struct.Struct('<H7hH')
        self.EarduinoDatas = [0,0,0,0,0,0,0]

        self.buf = ''

    def ser_open(self):
        try:
            self.arduino_ser = serial.Serial(arduino_port, 115200, timeout=1)
            #self.logger.info(self.ahrs_ser.portstr+' open successfully')
            return True
        except(serial.serialutil.SerialException):
            #self.logger.info('could not open port: '+ahrs_port)
            raise

    def update(self):
        if self.ser_open_flag is True:
            self.read_data()
            self.arduino_ser.write(self.send_data())

    def read_data(self):
        self.buf += self.arduino_ser.read(18-len(self.buf))
        print 'arduino read:',binascii.hexlify(self.buf)
        idx = self.buf.find(self.header)
        if idx < 0:
            self.buf = ''
            print ('ReadError: header not found, discard buffer')
            return
        elif idx > 0:
            self.buf = self.buf[idx:]
            print ('ReadError: header not at start, discard bytes before header')
            return
        if len(self.buf) < 18:
            print ('ReadError: not enough data')
            return

        #testBety = self.buf[0:9]

        datas = self.recvDataFst.unpack(self.buf)
        #print datas
        crc16Num = crc16(self.buf[2:-2])


        if datas[-1] != crc16Num:
            self.buf = self.buf[2:]
            print ('ReadError: ckcum error, discard first 2 bytes')
            return

        self.EarduinoDatas = datas[1:-1]
        print 'arduino read data:',self.EarduinoDatas

        self.buf = ''

    def send_data(self):
        header = '\xff\x01'
        tmp = self.fst.pack(motor,rudder,sail,pcCtrl)
        crc_code = struct.pack('!H', crc16(tmp))
        print 'send data:',binascii.hexlify(tmp), type(tmp)
        tmp = header + tmp
        tmp = tmp + crc_code
        #print binascii.hexlify(tmp)
        return tmp

    def close(self):
        self.arduino_ser.close()


class SensorListener:
    def __init__(self,nodeName,topicName):
        self.NodeName = nodeName
        self.TopicName = topicName
        rospy.init_node(self.NodeName, anonymous=True)
        self.pub = rospy.Publisher('arduino', Arduino_msg, queue_size=5)
        self.r = rospy.Rate(10)
        self.arduino = Arduino()
        self.arduinomsg = Arduino_msg()
        self.listener()


    def listener(self):

        #rospy.Subscriber("Ahrs", Ahrs_msg, callback)
        rospy.Subscriber(self.TopicName, Mach_msg, callback)

        # spin() simply keeps python from exiting until this node is stopped
        while 1:
        #while not rospy.is_shutdown():
            self.arduino.update()
            self.arduinomsg.header.stamp = rospy.Time.now()
            self.arduinomsg.header.frame_id = 'arduino'
            #self.arduinomsg.timestamp = rospy.get_time()
            self.arduinomsg.readMark = self.arduino.EarduinoDatas[0]
            self.arduinomsg.autoFlag = self.arduino.EarduinoDatas[1]
            self.arduinomsg.motor = self.arduino.EarduinoDatas[2]
            self.arduinomsg.rudder = self.arduino.EarduinoDatas[3]
            self.arduinomsg.sail = self.arduino.EarduinoDatas[4]
            self.arduinomsg.voltage1 = self.arduino.EarduinoDatas[5]
            self.arduinomsg.voltage2 = self.arduino.EarduinoDatas[6]
            self.pub.publish(self.arduinomsg)
            #self.r.sleep()
            # float64 timestamp
            # float64 readMark
            # float64 autoFlag
            # float64 motor
            # float64 rudder
            # float64 sail
            # float64 voltage1
            # float64 voltage2




def callback(data):
    global motor, rudder, sail, pcCtrl
    #print ('start')
    #rospy.loginfo("I heard %f", data.roll)
    motor = 50
    #max~min 40~-40 130~50
    rudder = int(data.rudder*57.3)+90
    #max~min 90-0 77~50
    sail = int(abs(data.sail)*57.3/3.3)+50

    pcCtrl = int(data.PCCtrl)

    if motor > 100:
        motor = 100
    if motor < 0:
        motor = 0

    if rudder > 130:
        rudder = 130
    if rudder < 50:
        rudder = 50
    if sail > 130:
        sail = 130
    if sail < 50:
        sail = 50
   # rospy.loginfo("subscriber successfully")

    #print "ros send"
    #print data.motor,data.rudder,data.sail,data.pcCtrl
    print 'get mach msg:',motor,rudder,sail,pcCtrl


def talker():#ros message publish

    SensorListener('arduinoTest','mach')

    rospy.spin()



if __name__ == '__main__':



    talker()

