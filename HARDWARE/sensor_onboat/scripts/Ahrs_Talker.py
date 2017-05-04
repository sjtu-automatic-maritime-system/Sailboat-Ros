#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
#include "sailboat_message/Mach_msg.h"
from   sailboat_message.msg import Ahrs_msg
import serial
import struct
import logging

Data_Show = False

ahrs_port = '/dev/ahrs'

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
        self.fst = struct.Struct("<9fH")
        self.buf = ''
        self.attrs = ['Roll', 'Pitch', 'Yaw', 'gx', 'gy', 'gz',
                      'wx', 'wy', 'wz', 'devicestatus']

    def ser_open(self):
        try:
            self.ahrs_ser = serial.Serial(ahrs_port, 115200, timeout=1)
            self.logger.info(self.ahrs_ser.portstr+' open successfully')
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
        self.buf += self.ahrs_ser.read(44-len(self.buf))
        #print(self.buf)
        idx = self.buf.find(self.header)
        if idx < 0:
            self.buf = ''
            self.logger.info('ReadError: header not found, discard buffer')
            return
        elif idx > 0:
            self.buf = self.buf[idx:]
            self.logger.info('ReadError: header not at start, discard bytes before header')
            return
        if len(self.buf) < 44:
            self.logger.info('ReadError: not enough data')
            return

        #testBety = self.buf[0:9]

        datas = self.fst.unpack(self.buf[5:43])
        testCRC = crc16(self.buf[2:41])
        testHexCRC = hex(testCRC)
        HexCRC = hex(datas[9])
        if len(testHexCRC) != 6 :
            testHexCRC = testHexCRC[0:2]+'0'*(6-len(testHexCRC))+testHexCRC[2:len(testHexCRC)]
        if len(HexCRC) != 6 :
            HexCRC = HexCRC[0:2]+'0'*(6-len(HexCRC))+HexCRC[2:len(HexCRC)]
        jugCRC = testHexCRC[0:2]+testHexCRC[4:6]+testHexCRC[2:4]
        
        #print ("jugCRC = ", testHexCRC)
        #print (type(testHexCRC))
        #print ("HexCRC = ", HexCRC)
        if jugCRC  != HexCRC :
            self.buf = self.buf[2:]
            self.logger.info('ReadError: ckcum error, discard first 2 bytes')
            return
        
        self.Roll = datas[0]
        self.Pitch = datas[1]
        self.Yaw = datas[2]
        self.gx = datas[3]
        self.gy = datas[4]
        self.gz = datas[5]
        self.wx = datas[6]
        self.wy = datas[7]
        self.wz = datas[8]
        
        #print (type(self.buf))
        #print ('bety = ', testBety)
        #print ('Roll = ', datas[0])
        #abc = hex(datas[0])
        #print ('Roll = ', abc)

        self.buf = ''

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




def talker():#ros message publish
    pub = rospy.Publisher('Ahrs', Ahrs_msg, queue_size=5)
    rospy.init_node('Ahrs_Talker', anonymous=True)
    rate = rospy.Rate(20) # 10hz

    ahrs = AHRS()
    ahrs_msg = Ahrs_msg()
    for ii in range(10):
        ahrs.update()
    try:
        while not rospy.is_shutdown():
            ahrs.update()

            ahrs_msg.timestamp = rospy.get_time()

            ahrs_msg.roll = ahrs.Roll
            ahrs_msg.pitch = ahrs.Pitch
            ahrs_msg.yaw = ahrs.Yaw
            ahrs_msg.gx = ahrs.gx
            ahrs_msg.gy = ahrs.gy
            ahrs_msg.gz = ahrs.gz
            ahrs_msg.wx = ahrs.wx
            ahrs_msg.wy = ahrs.wy
            ahrs_msg.wz  = ahrs.wz
            #show data
            rospy.loginfo(ahrs_msg.roll)
            pub.publish(ahrs_msg)
            rate.sleep()            
    except rospy.ROSInterruptException:
        pass
    finally:
        ahrs.close()


if __name__ == '__main__':
    talker()

