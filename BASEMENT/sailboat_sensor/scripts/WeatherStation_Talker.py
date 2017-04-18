#!/usr/bin/python
# -*- coding: utf-8 -*-

from math import *
import serial
import struct
import logging
import time

import rospy
from std_msgs.msg import String
from sailboat_message.msg import WTST_msg


# if using serial port, set it something like to "COM1" or "/dev/tty1"
WTST_URL = "/dev/wtrt"
# serial port baudrate
BAUDRATE = 4800
BAUDRATE_change=38400
# connection timeout in seconds
TIMEOUT = 2

# commands sent to GNSS receiver
INIT_COMMANDS = """$PAMTC,BAUD,38400
"""
# XOR checksum 
# example data = "$WIMWV,43.1,R,0.4,N,A"  
# rentun Types of int 
def xor_BCC(data):
    result = ord(data[1])
    for i in range(2,len(data)):
         result ^= ord(data[i])
    return result

def console_logger(name):
    # create logger
    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)
    # create console handler and set level to debug
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    # create formatter
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    # add formatter to ch
    ch.setFormatter(formatter)
    # add ch to logger
    logger.addHandler(ch)
    return logger

class WTST_ERROR(Exception):
    pass

class WTST:
    def __init__(self,url,baudrate,timeout,cmds):
        self.url = url
        self.logger = console_logger('WTST')
        self.cmds = cmds
        self.ser = serial.serial_for_url(url,do_not_open=True, baudrate=baudrate, timeout=timeout)
        self.open()

    def close(self):
        self.logger.info('WTST close: '+self.url)
        self.ser.close()

    def open(self):
        self.logger.info('WTST open: '+self.url)
        self.line = ''
        self.ser.open()
        #if self.cmds!=None and self.cmds!= "":
        self.ser.write(self.cmds.replace("\r","").replace("\n","\r\n"))
        time.sleep(2)
        #self.ser.write(self.cmds)
        print('send cmds')
        self.ser.baudrate = 38400
        print('set baudrate')


    def update(self):
        l = self.ser.readline()
        #print l
        if l == '':
            self.logger.warning('WTST timeout, reconnect')
            self.close()
            self.open()
        self.line = self.line + l
        if self.line.endswith('\n'):
            # a new line received
            self.parse_line()
            self.line = ''

    def parse_line(self):
        if not self.line.startswith('$'):
            return
        self.line = self.line.rstrip('\r\n')
        # XOR checksum  
        self.line_list = self.line.split('*')
        result = xor_BCC(self.line_list[0])
        #print (int(self.line_list[1], 16))
        #print ('result = ',result)
        if result != int(self.line_list[1], 16):
            self.logger.warning('invalid novatel cksum: '+self.line)
            return
        #print (self.line_list)
        #parse
        self.parse_content()

    def parse_content(self):
        self.parsedata = self.line_list[0].split(',')
        self.parsehead = self.parsedata[0]
        #print (self.parsehead)
        # $GPGGA,,,,,,0,,,,,,,,*66  
        # $GPVTG,,,,,,,,,N*30
        # $GPZDA, , , , *48
        # $WIMDA,29.9227,I,1.0133,B,21.2,C,,,,,,,,,,,,,,*34 
        # $WIMWV,43.1,R,0.4,N,A*11
        # $TIROT,-22.0,A*26
        # $YXXDR,C,,C,WCHR,C,,C,WCHT,C,,C,HINX,P,1.0133,B,STNP*4B
        # $YXXDR*4F
        if self.parsehead == '$GPGGA':
            self.parse_GPGGA()
        elif self.parsehead == '$GPVTG':
            self.parse_GPVTG()
        elif self.parsehead == '$GPZDA':
            self.parse_GPZDA()
        elif self.parsehead == '$WIMDA':
            self.parse_WIMDA()
        elif self.parsehead == '$WIMWV':
            self.parse_WIMWV()
        elif self.parsehead == '$TIROT':
            self.parse_TIROT()
        elif self.parsehead == '$YXXDR':
            self.parse_YXXDR()

    def parse_GPGGA(self):
        if len(self.parsedata) != 15:
            raise WTST_ERROR('invalid GPGGA '+self.line_list[0])
        if int(self.parsedata[6]) == 0:
            self.GPGGAFlag = 0
            self.Latitude = 0
            self.Longitude = 0
            self.Altitude = 0
            self.NumSata = 0
        else:
            self.GPGGAFlag = int(self.parsedata[6])
            self.Latitude = float(self.parsedata[2])
            self.Longitude = float(self.parsedata[4])
            self.Altitude = float(self.parsedata[9])
            self.NumSata = int(self.parsedata[7])
        #print (self.GPGGAFlag, self.Latitude, self.Longitude, self.Altitude, self.NumSata)

    def parse_GPVTG(self):
        if len(self.parsedata) != 10:
            raise WTST_ERROR('invalid GPVTG '+self.line_list[0])
        if self.parsedata[9] == 'N' :
            self.GPVTGFlag = 0
            self.DegreeTrue = 0
            self.DegreeMagmetic = 0
            self.SpeedKnots = 0
            self.SpeedKmhr = 0
        else:
            self.GPVTGFlag = 1
            self.DegreeTrue = float(self.parsedata[1])
            self.DegreeMagmetic = float(self.parsedata[3])
            self.SpeedKnots = float(self.parsedata[5])
            self.SpeedKmhr = float(self.parsedata[7])
        #print (self.GPVTGFlag, self.DegreeTrue, self.DegreeMagmetic, self.SpeedKnots, self.SpeedKmhr)

    def parse_GPZDA(self):
        if len(self.parsedata) != 5: # different from the UserManual
            raise WTST_ERROR('invalid GPZDA '+self.line_list[0])
        if  self.parsedata[1] == '':
            self.GPZDAFlag = 0
            self.UTCTime = 0
            self.UTCDay = 0
            self.UTCMonth = 0
        else:
            self.GPZDAFlag = 1
            self.UTCTime = int(self.parsedata[1])
            self.UTCDay = int(self.parsedata[2])
            self.UTCMonth = int(self.parsedata[3])
        #print (self.GPZDAFlag,self.UTCTime,self.UTCDay,self.UTCMonth)

    def parse_WIMDA(self):
        if len(self.parsedata) != 21:
            raise WTST_ERROR('invalid WIMDA '+self.line_list[0])
        if self.parsedata[1] == '':
            self.WIMDAFlag = 0
            self.BarometricPressureMercury = 0
            self.AirTemperature = 0
        else:
            self.WIMDAFlag = 1
            self.BarometricPressureMercury = float(self.parsedata[1])
            self.AirTemperature = float(self.parsedata[5])
        if self.GPVTGFlag == 0:
            self.WindDirectionTrue = 0
            self.WindDirectionMagnetic = 0
            self.WindSpeedKnots = 0
            self.WindSpeedMs = 0
        else:
            self.WindDirectionTrue = float(self.parsedata[13])
            self.WindDirectionMagnetic = float(self.parsedata[15]) 
            self.WindSpeedKnots = float(self.parsedata[17])
            self.WindSpeedMs = float(self.parsedata[19])
        #print(self.WIMDAFlag, self.BarometricPressureMercury, self.AirTemperature,self.WindDirectionTrue, self.WindDirectionMagnetic, self.WindSpeedKnots, self.WindSpeedMs)

    def parse_WIMWV(self):
        if len(self.parsedata) != 6:
            raise WTST_ERROR('invalid WIMWV '+self.line_list[0])
        if self.parsedata[5] == 'V':
            self.WIMWVFlag = 0
            self.WindAngle = 0
            self.WindReference = ''
            self.WindSpeed = 0
            self.WindSpeedUnit = ''
        else:
            self.WIMWVFlag = 1
            self.WindAngle = float(self.parsedata[1])
            self.WindReference = self.parsedata[2]
            self.WindSpeed = float(self.parsedata[3])
            self.WindSpeedUnit = self.parsedata[4]
        #print(self.WIMWVFlag, self.WindAngle, self.WindReference, self.WindSpeed, self.WindSpeedUnit)

    def parse_TIROT(self):
        #print ('len(TIROT) = ',len(self.parsedata[0]))
        if len(self.parsedata) != 3:
            raise WTST_ERROR('invalid TIROT '+self.line_list[0])
        if self.parsedata[2] == 'V':
            self.TIROTFlag = 0
            #degree per minute
            self.RateTurn = 0
        else:
            self.TIROTFlag = 1
            self.RateTurn = float(self.parsedata[1])
        #print (self.TIROTFlag, self.RateTurn)

    def parse_YXXDR(self):
        self.YXXDR = 1
        #print('YXXDR')
        #if len(self.parsedata) != 17 :
            #raise WTST_ERROR('invalid YXXDR '+self.line_list[0])

def talker():#ros message publish
    pub = rospy.Publisher('WTST', WTST_msg, queue_size=5)
    rospy.init_node('WTST_Talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    wtst = WTST(WTST_URL, BAUDRATE, TIMEOUT,INIT_COMMANDS) 
    wtst_msg = WTST_msg()
    try:
        for ii in range(32):
            wtst.update()
        while not rospy.is_shutdown():
            wtst.update()
            wtst.update()
            wtst.update()
            wtst.update()
            wtst.update()
            wtst.update()
            wtst.update()
            wtst.update()

            wtst_msg.GPGGAFlag = wtst.GPGGAFlag
            wtst_msg.Latitude = wtst.Latitude
            wtst_msg.Longitude = wtst.Longitude
            wtst_msg.Altitude = wtst.Altitude
            wtst_msg.NumSata = wtst.NumSata
            wtst_msg.GPVTGFlag = wtst.GPVTGFlag
            wtst_msg.DegreeTrue = wtst.DegreeTrue
            wtst_msg.DegreeMagmetic = wtst.DegreeMagmetic
            wtst_msg.SpeedKnots = wtst.SpeedKnots
            wtst_msg.SpeedKmhr = wtst.SpeedKmhr
            wtst_msg.GPZDAFlag = wtst.GPZDAFlag
            wtst_msg.UTCTime = wtst.UTCTime
            wtst_msg.UTCDay = wtst.UTCDay
            wtst_msg.UTCMonth = wtst.UTCMonth
            wtst_msg.WIMDAFlag = wtst.WIMDAFlag
            wtst_msg.BarPreMer = wtst.BarometricPressureMercury
            wtst_msg.AirTemperature = wtst.AirTemperature
            wtst_msg.WindDirectionTrue = wtst.WindDirectionTrue
            wtst_msg.WindDirectionMagnetic = wtst.WindDirectionMagnetic
            wtst_msg.WindSpeedKnots = wtst.WindSpeedKnots
            wtst_msg.WindSpeedMs = wtst.WindSpeedMs
            wtst_msg.WIMWVFlag = wtst.WIMWVFlag
            wtst_msg.WindAngle = wtst.WindAngle
            wtst_msg.WindReference = wtst.WindReference
            wtst_msg.WindSpeed = wtst.WindSpeed
            wtst_msg.WindSpeedUnit = wtst.WindSpeedUnit
            wtst_msg.TIROTFlag = wtst.TIROTFlag
            wtst_msg.RateTurn = wtst.RateTurn
            rospy.loginfo(wtst_msg.WindAngle)
            pub.publish(wtst_msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        wtst.close()


if __name__ == '__main__':
    talker()