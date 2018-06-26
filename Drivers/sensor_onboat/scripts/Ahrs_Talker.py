#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
#include "sailboat_message/Mach_msg.h"
from sailboat_message.msg import Ahrs_msg
import serial
import struct
import logging
import time

Data_Show = False

#ahrs_port = '/dev/ttyUSB0'
ahrs_port = '/dev/ahrs'
# ahrs_port='/dev/serial/by-id/usb-Silicon_Labs_SBG_Systems_-_UsbToUart_001000929-if00-port0'

def hexShow(argv):
    result = ''
    hLen = len(argv)
    for i in xrange(hLen):
        hvol = ord(argv[i])
        hhex = '%02x' % hvol
        result += hhex+' '
    print 'hexShow:', result


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


class AbortProgram(Exception):
    pass


def console_logger(name):
    # create logger
    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)
    # create console handler and set level to debug
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    # create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    # add formatter to ch
    ch.setFormatter(formatter)
    # add ch to logger
    logger.addHandler(ch)
    return logger


class AHRS():
    def __init__(self):
        self.logger = console_logger('AHRS')
        self.ser_open_flag = self.ser_open()
        self.DataShow_count = 0
        self.header = chr(0xff)+chr(0x02)
        self.fst = struct.Struct("<22fL5f")
        self.buf = ''
        self.attrs = ['Roll', 'Pitch', 'Yaw', 'gx', 'gy', 'gz',
                      'ax', 'ay', 'az', 'devicestatus']

    def ser_open(self):
        try:
            self.ahrs_ser = serial.Serial(ahrs_port, 115200, timeout=1)
            self.logger.info(self.ahrs_ser.portstr+' open successfully')
            time.sleep(1)
            return True
        except(serial.serialutil.SerialException):
            self.logger.info('could not open port: '+ahrs_port)
            raise

    def update(self):
        if self.ser_open_flag is True:
            self.read_data()
        if not Data_Show:
            return
        self.DataInfoShow()

    def read_data(self):
        #120*2-1 = 239
        #2+3+112+2+1
        n = self.ahrs_ser.inWaiting()
        self.buf += self.ahrs_ser.read(n)
        self.buf = self.buf[-239:]

        # self.buf = self.ahrs_ser.read(120)
        # print(self.buf)
        idx = self.buf.find(self.header)
        if idx < 0:
            self.buf = ''
            self.logger.info('ReadError: header not found, discard buffer')
            return
        elif idx > 0:
            self.buf = self.buf[idx:]
            self.logger.info('ReadError: header not at start, discard bytes before header')
            return
        if len(self.buf) < 119:
            self.logger.info('ReadError: not enough data')
            return

        #testBety = self.buf[0:9]

        datas = self.fst.unpack(self.buf[5:117])
        fff=struct.Struct(">H")
        crcnum=fff.unpack(self.buf[117:119])
        if crc16(self.buf[2:117])!=crcnum[0]:
            self.buf = self.buf[2:]
            self.logger.info('ReadError: ckcum error, discard first 2 bytes')
            return
        
        self.Roll = datas[4]
        self.Pitch = datas[5]
        self.Yaw = datas[6]
        self.gx = datas[16]
        self.gy = datas[17]
        self.gz = datas[18]
        self.ax = datas[19]
        self.ay = datas[20]
        self.az = datas[21]
        self.heave=datas[27]
        self.accuracy=datas[23]
        
        self.buf = self.buf[120:]
        # print(len(datas))
        print(self.Roll,self.Pitch,self.Yaw,self.gx,self.gy,self.gz,self.ax,self.ay,self.az,self.heave,self.accuracy)

    def DataInfoShow(self):
        self.DataShow_count += 1
        if self.DataShow_count < 5:
            return
        self.DataShow_count = 0
        self.logger.info('AHRS pub data:')
        print ('roll, yaw, yaw_rate, devicestatus: ',
               getattr(self, 'Roll', 0), getattr(self, 'Yaw', 0),
               getattr(self, 'gz', 0), getattr(self, 'devicestatus', 0))

    def close(self):
        self.ahrs_ser.close()

    def isset(self,dataname):
        try:
            type (eval('self.'+dataname))
        except:
            return 0
        else:
            return 1

class dataWrapper:
    """docstring for dataWrapper"""
    def __init__(self):
        self.Roll  = 'Roll'
        self.Pitch = 'Pitch'
        self.Yaw   = 'Yaw'
        self.gx    = 'gx'
        self.gy    = 'gy'
        self.gz    = 'gz'
        self.ax    = 'ax'
        self.ay    = 'ay'
        self.az    = 'az'
        self.heave='heave'
        self.accuracy='accuracy'


    def pubData(self,msg,ahrs):
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'AHRS'

        if ahrs.isset(self.Roll):
            msg.roll = ahrs.Roll
        if ahrs.isset(self.Pitch):
            msg.pitch = ahrs.Pitch
        if ahrs.isset(self.Yaw):
            msg.yaw = ahrs.Yaw
        if ahrs.isset(self.gx):
            msg.gx = ahrs.gx
        if ahrs.isset(self.gy):
            msg.gy = ahrs.gy
        if ahrs.isset(self.gz):
            msg.gz = ahrs.gz
        if ahrs.isset(self.ax):
            msg.ax = ahrs.ax
        if ahrs.isset(self.ay):
            msg.ay = ahrs.ay
        if ahrs.isset(self.az):
            msg.az = ahrs.az
        if ahrs.isset(self.heave):
            msg.heave = ahrs.heave
        if ahrs.isset(self.accuracy):
            msg.accuracy = ahrs.accuracy
        return msg


def talker():#ros message publish
    pub = rospy.Publisher('ahrs', Ahrs_msg, queue_size=5)
    rospy.init_node('ahrs_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hzahrs_

    ahrs = AHRS()
    msg = Ahrs_msg()

    datawrapper = dataWrapper()

    for ii in range(10):
        ahrs.update()
    try:
        while not rospy.is_shutdown():
            ahrs.update()
            #ahrs_msg.timestamp = rospy.get_time()

            ahrs_msg= datawrapper.pubData(msg,ahrs)
            #show data
            #rospy.loginfo(ahrs_msg.roll)
            pub.publish(ahrs_msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        ahrs.close()


if __name__ == '__main__':
    talker()

