#!/usr/bin/python

import serial
import time
import rospy
import logging
from sailboat_message.msg import Radar_msg
from sailboat_message.msg import Pointinfo
from std_msgs.msg import String
#radar_port = '/dev/radar'
radar_port = '/dev/ttyUSB2'
#ser = serial.Serial('/dev/ttyUSB0',115200,timeout=0.5)
# text = []
# num = []
#   points = []
class Pointsinfo:
    def __init__(self):
        self.id = -1
        self.verl = -1
        self.azimuth = -1
        self.range = -1
class PointtoString:
    def getDescription(self):
        return ",".join("{}={}".format(key, getattr(self, key)) for key in self.__dict__.keys())
    def __str__(self):
        return "{}->({})".format(self.__class__.__name__,self.getDescription())
    __repr__ = __str__
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

class RADAR(PointtoString, Pointsinfo):
    def __init__(self):
        self.logger = console_logger('RADAR')
        self.ser_open_flag = self.ser_open()
        self.buf = ''
        self.DataShow_count = 0
        self.Pointsnumbers = -1
        self.pointlist = [] # container of points info
        self.header = chr(0xaa)+chr(0xaa)  # header
        self.header1 = chr(0x0b)+chr(0x07) # check how many points
        self.header2 = chr(0x0c)+chr(0x07) # points info
        self.header3 = chr(0x0a)+chr(0x06) # system status
        self.header4 = chr(0x06) # the str after '\n'

    def ser_open(self):
        try:
            self.radar_ser = serial.Serial(radar_port, 115200, timeout=1)
            self.logger.info(self.radar_ser.portstr + ' open successfully')
            return True
        except(serial.serialutil.SerialException):
            self.logger.info('could not open port: ' + radar_port)

    def update(self):
        self.pointlist = []
        if self.ser_open_flag is True:
            self.read_data()
        #if not Data_Showi:
        #    return
        self.DataInfoShow()

    def read_data(self):
        #self.buf += self.radar_ser.read(14 - len(self.buf))
        # print(self.buf)
        self.buf = self.radar_ser.readline() #read whole line until '\n'idx = self.buf.find(self.header)
        if self.buf.find(header4) != 0:
            self.logger.info("ReadError: wrong message!")
            return
        idx = self.buf.find(self.header1) #flag of points number
        if idx < 0:
            self.logger.info('ReadError: wrong message!')
            return
        tempbuf = self.buf[idx:] #throw message before flag
        self.Pointsnumber = ord(tempbuf[2]) #calculate points number
        for i in range(self.Pointsnumbers):
            idx = tempbuf.find(self.header2) # find flag of points info
            tempbuf = tempbuf[idx:]
            Pointinfo = []
            point = Pointsinfo()
            point.id = ord(tempbuf[2])
            Pointinfo.append(point.id)
            point.verl = (ord(tempbuf[7])*256 + ord(tempbuf[8]))*0.05 - 35
            Pointinfo.append(point.verl)
            point.azimuth = ord(tempbuf[6])*2 - 50
            Pointinfo.append(point.azimuth)
            point.range = (ord(tempbuf[4])*0x100 + ord(self.buf[5]))*0.01 #calculate points info
            Pointinfo.append(point.range)# wrap massage info into a list
            self.pointlist.append(Pointinfo) # put list into self.pointlist
            tempbuf = tempbuf[13:] #next point info

    def DataInfoShow(self):
        self.DataShow_count += 1
        if self.DataShow_count < 5:
           return
        self.DataShow_count = 0
        self.logger.info('RADAR pub data:')
        print('id, verl, azimuth, range: ',
                  self.Id, self.Verl, self.Azimuth, self.Range)

    def close(self):
        self.radar_ser.close()

    def isset(self, dataname):
        try:
            type(eval('self.' + dataname))
        except:
            return 0
        else:
            return 1

class dataWrapper:
    def __init__(self):
        self.Pointsnumber = 'Pointsnumber'
        self.pointlist = 'pointlist'
    def pubData(self, msg, radar):
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'RADAR'
        if radar.isset(self.Pointsnumber):
            msg.Pointsnumber = radar.Pointsnumber
        if radar.isset(self.pointlist):
            msg.pointlist = radar.pointlist
        return msg

def talker():#ros message publisher
    rospy.init_node('radar_talker', anonymous=True)
    pub = rospy.Publisher('radar', Radar_msg, queue_size=5)

    rate = rospy.Rate(50) # 10hz radar

    radar = RADAR()
    msg = Radar_msg()

    datawrapper = dataWrapper()

    for ii in range(10):
        radar.update()
    try:
        while not rospy.is_shutdown():
            radar.update()
            #ahrs_msg.timestamp = rospy.get_time()

            radar_msg= datawrapper.pubData(msg,radar)
            #show data
            #rospy.loginfo(ahrs_msg.roll)
            pub.publish(radar_msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        radar.close()

if __name__ == '__main__':
    talker()
