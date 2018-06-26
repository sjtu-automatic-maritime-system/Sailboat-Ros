import serial
import time
import rospy
import logging
from sailboat_message.msg import Radar_msg
from std_msgs.msg import String
#radar_port = '/dev/radar'
radar_port = '/dev/ttyUSB2'
#ser = serial.Serial('/dev/ttyUSB0',115200,timeout=0.5)
# text = []
# num = []
# points = []

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

class RADAR(PointtoString):
    def __init__(self):
        self.logger = console_logger('RADAR')
        self.ser_open_flag = self.ser_open()
        self.buf = ''
        self.DataShow_count = 0
        self.Id = -1
        self.Rcs = -1
        self.Verl = -1
        self.Azimuth = -1
        self.Range = -1
        self.header = chr(0xaa)+chr(0xaa)
        self.header1 = chr(0x0b)+chr(0x07)
        self.header2 = chr(0x0c)+chr(0x07)
        self.header3 = chr(0x0a)+chr(0x06)

    def ser_open(self):
        try:
            self.radar_ser = serial.Serial(radar_port, 115200, timeout=1)
            self.logger.info(self.radar_ser.portstr + ' open successfully')
            return True
        except(serial.serialutil.SerialException):
            self.logger.info('could not open port: ' + radar_port)
        raise

    def update(self):
        if self.ser_open_flag is True:
            self.read_data()
        #if not Data_Showi:
        #    return
        self.DataInfoShow()

    def read_data(self):
        self.buf += self.radar_ser.read(14 - len(self.buf))
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
        if len(self.buf) < 14:
            self.logger.info('ReadError: not enough data')
            return

        if self.buf.find(self.header2) >= 0:
            self.Id = ord(self.buf[4])
            self.Rcs = ord(self.buf[5])*0.5 - 50
            self.Verl = (ord(self.buf[9]) * 256 + ord(self.buf[10])) * 0.05 - 35
            self.Azimuth = ord(self.buf[8]) * 2 - 50
            self.Range = (ord(self.buf[6]) * 0x100 + ord(self.buf[7])) * 0.01
            self.buf = ''

        if self.buf.find(self.header1) >= 0:
            self.buf = ''

        if self.buf.find(self.header3) >= 0:
            self.buf = ''

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
        self.Id = 'id'
        self.Verl = 'verl'
        self.Azimuth = 'azimuth'
        self.Range = 'range'
    def pubData(self, msg, radar):
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'RADAR'

        if radar.isset(self.Id):
            msg.id = radar.Id
        if radar.isset(self.Range):
            msg.range = radar.Range
        if radar.isset(self.Verl):
            msg.verl = radar.Verl
        if radar.isset(self.Azimuth):
            msg.azimuth = radar.Azimuth
        return msg

def talker():#ros message publish
    pub = rospy.Publisher('radar', Radar_msg, queue_size=5)
    rospy.init_node('radar_talker', anonymous=True)
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
# #ser.bytesize= 8
# #print(ser.bytesize)
# for i in range(500):
#     s = ser.read(2)
#     if s[0] == 0xaa and s[1] == 0xaa:
#         line = ser.read(12)
#         t = time.time()
#         text.append(int(round(t * 1000)))
#         text.append(line)
#         if line[0] == 0x0b and line[1] == 0x07:
#             num.append(line[2])
#             num.append(int(round(t * 1000)))
#         if line[0] == 0x0c and line[1] == 0x07:
#             id = int(line[2])
#             rcs = int(line[3])*0.5 - 50
#             verl = (line[7]*256 + line[8])*0.05 - 35
#             azimuth = line[6]*2 - 50
#             ranges = (line[4]*0x100 + line[5])*0.01
#             p = Point(id, rcs, verl, azimuth, ranges)
#             points.append(p)
#             #points.append(int(round(t * 1000)))
# #print(text)
# #print(num)
# print(points)
# #print(type(points[1]))
# points_num = max(p.Id for p in points)
# # print(type(points_num))
# #print(points_num)
# velocity_dict = {}
# ranges_dict = {}
# angel_dict = {}
# rcs_dict = {}
# for k in range(1, points_num+1):
#     rcs_dict[k] = []
#     velocity_dict[k] = []
#     ranges_dict[k] = []
#     angel_dict[k] = []
#     for point in points:
#         if point.Id == k:
#             rcs_dict[k].append(point.Rcs)
#             velocity_dict[k].append(point.Verl)
#             ranges_dict[k].append(point.Range)
#             angel_dict[k].append(point.Azimuth)
# print(velocity_dict)
# print(ranges_dict)
# print(angel_dict)
# print(rcs_dict)
# #print(type(line))8
if __name__ == '__main__':
    talker()
