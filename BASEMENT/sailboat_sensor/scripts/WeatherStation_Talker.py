#!/usr/bin/python
# -*- coding: utf-8 -*-

from math import *
import serial
# import struct
# import logging
import time

import rospy
# from std_msgs.msg import String
from sailboat_message.msg import WTST_msg
from sailboat_message.msg import WTST_Pro_msg


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

# lat/lon of original point, use to caculate posx/posy (north and east are positive)
ORIGIN_LAT = 31.0231632
ORIGIN_LON = 121.4251289

# XOR checksum 
# example data = "$WIMWV,43.1,R,0.4,N,A"  
# rentun Types of int 
def xor_BCC(data):
    result = ord(data[1])
    for i in range(2,len(data)):
         result ^= ord(data[i])
    return result

def d2r(d):
    return d/180.0*pi

class WTST_ERROR(Exception):
    pass

class WTST:
    def __init__(self,url,baudrate,timeout,cmds):
        self.url = url
        self.cmds = cmds
        self.ser = serial.serial_for_url(url,do_not_open=True, baudrate=baudrate, timeout=timeout)
        self.open()
        time.sleep(1)
        #self.close()
        #time.sleep(1)
        #self.open()
        #self.GPVTGFlag = 0

    def set_origin(self, lat,lon):
        self.lat = d2r(lat)
        self.lon = d2r(lon)

    def w84_calc_ne(self, lat2, lon2):
        lat1,lon1 = self.lat,self.lon
        lat2,lon2 = d2r(lat2),d2r(lon2)
        d_lat = lat2-lat1
        d_lon = lon2-lon1

        a = 6378137.0
        e_2 = 6.69437999014e-3
        r1 = a*(1-e_2)/(1-e_2*(sin(lat1))**2)**1.5
        r2 = a/sqrt(1-e_2*(sin(lat1))**2)

        north = r1*d_lat
        east = r2*cos(lat1)*d_lon
        return north,east

    def close(self):
        self.ser.close()
        rospy.loginfo('WTST close')

    def open(self):
        self.line = ''
        self.ser.open()
        rospy.loginfo('WTST open')
        #if self.cmds!=None and self.cmds!= "":
        self.ser.write(self.cmds.replace("\r","").replace("\n","\r\n"))
        time.sleep(2)
        #self.ser.write(self.cmds)
        rospy.loginfo('WTST send cmds')
        self.ser.baudrate = 38400
        rospy.loginfo('WTST set 38400 baudrate')


    def update(self):
        l = self.ser.readline()
        #print l
        if l == '':
            rospy.logwarn('WTST timeout, reconnect')
            self.close()
            self.open()
        self.line = self.line + l
        if self.line.endswith('\n'):
            # a new line received
            self.parse_line()
            self.line = ''

    def parse_line(self):
        if not self.line.startswith('$'):
            self.ser.flush()
            rospy.logwarn("header not started with '$'")
            #self.ser.close()
            #time.sleep(1)
            #self.ser.open()
            return
        self.line = self.line.rstrip('\r\n')
        # XOR checksum  
        self.line_list = self.line.split('*')
        result = xor_BCC(self.line_list[0])
        #print (int(self.line_list[1], 16))
        #print ('result = ',result)
        try:
            if result != int(self.line_list[1], 16):
                rospy.logwarn('invalid novatel cksum: '+self.line)
                return
        except:
            rospy.logwarn('invalid novatel cksum error')
            self.ser.close()
            time.sleep(1)
            self.ser.open()
        #print (self.line_list)
        #parse
        self.parse_content()

    def parse_content(self):
        self.parsedata = self.line_list[0].split(',')
        self.parsehead = self.parsedata[0]
        #print (self.parsehead)
        # $GPGGA,, , , , , 0,, , , , , , , *66
        # $HCHDT, 65.0, T * 1A
        # $WIMDA, 29.9168, I, 1.0131, B, 24.9, C,, , , , , , , , , , , , , *30
        # $GPVTG,, , , , , , , , N * 30
        # $HCHDG, 70.4, 0.0, E, 5.5, W * 63
        # $WIMWV,311.8,R,0.7,N,A*2F
        # $YXXDR, A, 5.3, D, PTCH, A, 2.7, D, ROLL * 5E
        if self.parsehead == '$GPGGA':
            self.parse_GPGGA()
        elif self.parsehead == '$GPVTG':
            self.parse_GPVTG()
        elif self.parsehead == '$HCHDG':
            self.parse_HCHDG()
        elif self.parsehead == '$HCHDT':
            self.parse_HCHDT()
        elif self.parsehead == '$WIMDA':
            self.parse_WIMDA()
        elif self.parsehead == '$WIMWV':
            self.parse_WIMWV()
        elif self.parsehead == '$YXXDR':
            self.parse_YXXDR()

    def parse_GPGGA(self):
        if len(self.parsedata) != 15:
            raise WTST_ERROR('invalid GPGGA '+self.line_list[0])
        if int(self.parsedata[6]) != 0:
            #     self.Latitude = 0
            #     self.Longitude = 0
            #     self.Altitude = 0
            #     self.NumSata = 0
            # GPS quality indicator:
            # 0 = Fix not available or invalid          1 = GPS SPS Mode, fix valid
            # 2 = Differential GPS, SPS Mode, fix valid 3 = GPS PPS Mode, fix valid
            # 4 = Real Time Kinematic (RTK)             5 = Float RTK
            # 6 = Estimated (dead reckoning) Mode       7 = Manual Input Mode
            # 8 = Simulator Mode
            self.GPSIndicator = int(self.parsedata[6])
            # Latitude, to the nearest .0001 minute
            self.Latitude = float(self.parsedata[2])
            # Longitude, to the nearest .0001 minute
            self.Longitude = float(self.parsedata[4])
            #
            self.PosX, self.PosY = self.w84_calc_ne(self.Latitude, self.Longitude)
            # Number of satellites in use, 0-12
            self.NumSata = int(self.parsedata[7])
        else:
            self.GPSIndicator = int(self.parsedata[6])
        #print (self.GPGGAFlag, self.Latitude, self.Longitude, self.Altitude, self.NumSata)

    def parse_GPVTG(self):
        if len(self.parsedata) != 10:
            raise WTST_ERROR('invalid GPVTG '+self.line_list[0])
        if self.parsedata[9] != 'N' :
            # self.DegreeTrue = 0
            # self.DegreeMagmetic = 0
            # self.SpeedKnots = 0
            # self.SpeedKmhr = 0
            # Mode indicator:
            # A = Autonomous mode    D = Differential mode  E = Estimated (dead reckoning) mode
            # M = Manual input mode  S = Simulator mode     N = Data not valid
            self.VTGIndicator = self.parsedata[9]
            # Course Over Ground, degrees True, to the nearest 0.1 degree
            self.DegreeTrue = float(self.parsedata[1])
            # Course Over Ground, degrees Magnetic, to the nearest 0.1 degree
            self.DegreeMagnetic = float(self.parsedata[3])
            # Speed Over Ground, knots, to the nearest 0.1 knot
            self.SpeedKnots = float(self.parsedata[5])
        else:
            self.VTGIndicator = self.parsedata[9]
            # Speed Over Ground, km/hr, to the nearest 0.1 km/hr
            #self.SpeedKmhr = float(self.parsedata[7])
        #print (self.GPVTGFlag, self.DegreeTrue, self.DegreeMagmetic, self.SpeedKnots, self.SpeedKmhr)

    def parse_HCHDT(self):
        if len(self.parsedata) != 3:
            raise WTST_ERROR('invalid HCHDT '+self.line_list[0])
        if self.parsedata[2] == 'T' :
            # Heading relative to True North, degrees
            self.HeadingTrueNorth = float(self.parsedata[1])
            #print(self.HeadingTrueNorth)

    def parse_HCHDG(self):
        if len(self.parsedata) != 6:
            raise WTST_ERROR('invalid HCHDG '+self.line_list[0])
        if self.parsedata[1] != '' :

            # Magnetic sensor heading, degrees, to the nearest 0.1 degree
            #print(self.parsedata[1])
            self.HeadingMagneticSenor = float(self.parsedata[1])
            # Magnetic deviation, degrees east or west, to the nearest 0.1 degree.
            self.MagneticDeviation = float(self.parsedata[2])
            # E or W
            self.DirectionDeviation = self.parsedata[3]
            # Magnetic variation, degrees east or west, to the nearest 0.1 degree.
            self.MagneticVariation = float(self.parsedata[4])
            # E or W
            self.DirectionVariation = self.parsedata[5]

    def parse_WIMDA(self):
        if len(self.parsedata) != 21:
            raise WTST_ERROR('invalid WIMDA '+self.line_list[0])
        if self.parsedata[1] != '':
            # Barometric pressure, inches of mercury, to the nearest 0.01 inch
            self.BarMercury = float(self.parsedata[1])
            # Air temperature, degrees C, to the nearest 0.1 degree C
            self.AirTemperature = float(self.parsedata[5])

        if self.parsedata[13] !='':
            # Wind direction, degrees True, to the nearest 0.1 degree
            self.WindDirectionTrue = float(self.parsedata[13])
            # Wind direction, degrees Magnetic, to the nearest 0.1 degree
            self.WindDirectionMagnetic = float(self.parsedata[15])
            # Wind speed, knots, to the nearest 0.1 knot
            self.WindSpeedKnots = float(self.parsedata[17])
            # Wind speed, meters per second, to the nearest 0.1 m/s
            #self.WindSpeedMs = float(self.parsedata[19])
        #print(self.WIMDAFlag, self.BarometricPressureMercury, self.AirTemperature,self.WindDirectionTrue, self.WindDirectionMagnetic, self.WindSpeedKnots, self.WindSpeedMs)

    def parse_WIMWV(self):
        if len(self.parsedata) != 6:
            raise WTST_ERROR('invalid WIMWV '+self.line_list[0])
        if self.parsedata[5] == 'A':
            # Status: A = data valid; V = data invalid
            self.MWVStatus = self.parsedata[5]
            # Wind angle, 0.0 to 359.9 degrees, in relation to the vessel’s bow/centerline, to the nearest 0.1 degree. If the data for this field is not valid, the field will be blank.
            self.WindAngle = float(self.parsedata[1])
            # R = Relative (apparent wind, as felt when standing on the moving ship)
            #self.WindReference = self.parsedata[2]
            #Wind speed, to the nearest tenth of a unit. If the data for this field is not valid, the field will be blank.
            #In the PB200 WeatherStation, this field always contains "N" (knots).
            self.WindSpeed = float(self.parsedata[3])
        else:
            self.MWVStatus = self.parsedata[5]
            #self.WindSpeedUnit = self.parsedata[4]
        #print(self.WIMWVFlag, self.WindAngle, self.WindReference, self.WindSpeed, self.WindSpeedUnit)


    def parse_YXXDR(self):
        if len(self.parsedata) == 9:
            #Pitch: oscillation of vessel about its latitudinal axis. Bow moving up is positive. Value reported to the nearest 0.1 degree.
            self.Pitch = float(self.parsedata[2])
            #Roll: oscillation of vessel about its longitudinal axis. Roll to the starboard is positive. Value reported to the nearest 0.1 degree.
            self.Roll = float(self.parsedata[6])
            #print(self.Pitch,self.Roll)

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
        self.GPSIndicator = 'GPSIndicator'
        self.Latitude = 'Latitude'
        self.Longitude ='Longitude'
        self.PosX = 'PosX'
        self.PosY = 'PosY'
        self.Roll = 'Roll'
        self.Pitch = 'Pitch'
        self.Yaw = 'HeadingTrueNorth'
        self.WindAngle = 'WindAngle'
        self.WindSpeed = 'WindSpeed'

        self.NumSata = 'NumSata'
        self.VTGIndicator = 'VTGIndicator'
        self.DegreeTrue = 'DegreeTrue'
        self.DegreeMagnetic = 'DegreeMagnetic'
        self.SpeedKnots = 'SpeedKnots'
        self.HeadingMagneticSenor = 'HeadingMagneticSenor'
        self.MagneticDeviation = 'MagneticDeviation'
        self.DirectionDeviation = 'DirectionDeviation'
        self.MagneticVariation = 'MagneticVariation'
        self.DirectionVariation = 'DirectionVariation'
        self.BarMercury = 'BarMercury'
        self.AirTemperature = 'AirTemperature'
        self.WindDirectionTrue = 'WindDirectionTrue'
        self.WindDirectionMagnetic = 'WindDirectionMagnetic'
        self.WindSpeedKnots = 'WindSpeedKnots'
        self.MWVStatus = 'MWVStatus'


    def pubData(self,msg,wtst):
        msg.timestamp = rospy.get_time()
        if wtst.isset(self.GPSIndicator):
            msg.GPSIndicator = wtst.GPSIndicator
        if wtst.isset(self.Latitude):
            msg.Latitude = wtst.Latitude
        if wtst.isset(self.Longitude):
            msg.Longitude = wtst.Longitude
        if wtst.isset(self.PosX):
            msg.PosX = wtst.PosX
        if wtst.isset(self.PosY):
            msg.PosY = wtst.PosY
        if wtst.isset(self.Roll):
            msg.Roll = wtst.Roll
        if wtst.isset(self.Pitch):
            msg.Pitch = wtst.Pitch
        if wtst.isset(self.Yaw):
            msg.Yaw = wtst.HeadingTrueNorth
        if wtst.isset(self.WindAngle):
            msg.WindAngle = wtst.WindAngle
        if wtst.isset(self.WindSpeed):
            msg.WindSpeed = wtst.WindSpeed
        return msg

    def pubProData(self,msgPro,wtst):
        if wtst.isset(self.GPSIndicator):
            msgPro.GPSIndicator = wtst.GPSIndicator
        if wtst.isset(self.Latitude):
            msgPro.Latitude = wtst.Latitude
        if wtst.isset(self.Longitude):
            msgPro.Longitude = wtst.Longitude
        if wtst.isset(self.PosX):
            msgPro.PosX = wtst.PosX
        if wtst.isset(self.PosY):
            msgPro.PosY = wtst.PosY

        if wtst.isset(self.NumSata):
            msgPro.NumSata = wtst.NumSata
        if wtst.isset(self.VTGIndicator):
            msgPro.VTGIndicator = wtst.VTGIndicator
        if wtst.isset(self.DegreeTrue):
            msgPro.DegreeTrue = wtst.DegreeTrue
        if wtst.isset(self.DegreeMagnetic):
            msgPro.DegreeMagnetic = wtst.DegreeMagnetic
        if wtst.isset(self.SpeedKnots):
            msgPro.SpeedKnots = wtst.SpeedKnots
        if wtst.isset(self.HeadingMagneticSenor):
            msgPro.HeadingMagneticSenor = wtst.HeadingMagneticSenor
        if wtst.isset(self.MagneticDeviation):
            msgPro.MagneticDeviation = wtst.MagneticDeviation
        if wtst.isset(self.DirectionDeviation):
            msgPro.DirectionDeviation = wtst.DirectionDeviation
        if wtst.isset(self.MagneticVariation):
            msgPro.MagneticVariation = wtst.MagneticVariation
        if wtst.isset(self.DirectionVariation):
            msgPro.DirectionVariation = wtst.DirectionVariation

        if wtst.isset(self.Roll):
            msgPro.Roll = wtst.Roll
            #print(wtst.Roll)
        if wtst.isset(self.Pitch):
            msgPro.Pitch = wtst.Pitch
        if wtst.isset(self.Yaw):
            msgPro.Yaw = wtst.HeadingTrueNorth
        if wtst.isset(self.WindAngle):
            msgPro.WindAngle = wtst.WindAngle
        if wtst.isset(self.WindSpeed):
            msgPro.WindSpeed = wtst.WindSpeed

        if wtst.isset(self.BarMercury):
            msgPro.BarMercury = wtst.BarMercury
        if wtst.isset(self.AirTemperature):
            msgPro.AirTemperature = wtst.AirTemperature
            #print(wtst.AirTemperature)
        if wtst.isset(self.WindDirectionTrue):
            msgPro.WindDirectionTrue = wtst.WindDirectionTrue
        if wtst.isset(self.WindDirectionMagnetic):
            msgPro.WindDirectionMagnetic = wtst.WindDirectionMagnetic
        if wtst.isset(self.WindSpeedKnots):
            msgPro.WindSpeedKnots = wtst.WindSpeedKnots
        if wtst.isset(self.MWVStatus):
            msgPro.MWVStatus = wtst.MWVStatus

        return msgPro



def talker():  # ros message publish

    pub = rospy.Publisher('WTST', WTST_msg, queue_size=1)
    #pub = rospy.Publisher('WTST', WTST_Pro_msg, queue_size=1)
    rospy.init_node('WTST_Talker', anonymous=True)
    rate = rospy.Rate(20)  # 20hz

    wtst = WTST(WTST_URL, BAUDRATE, TIMEOUT, INIT_COMMANDS)
    wtst.set_origin(ORIGIN_LAT, ORIGIN_LON)

    msg = WTST_msg()
    #msgPro = WTST_Pro_msg()
    datawrapper = dataWrapper()

    try:
        for ii in range(28):
            wtst.update()
        while not rospy.is_shutdown():
            wtst.update()
            wtst.update()
            wtst.update()
            wtst.update()
            wtst.update()
            wtst.update()
            wtst.update()

            wtst_msg = datawrapper.pubData(msg,wtst)
            #wtst_pro_msg = datawrapper.pubProData(msgPro,wtst)

            pub.publish(wtst_msg)
            #pub.publish(wtst_pro_msg)
            rate.sleep()
    except rospy.ROSInterruptException as e:
        print(e)
    finally:
        print('wsts closed!')
        wtst.close()


if __name__ == '__main__':
    talker()

