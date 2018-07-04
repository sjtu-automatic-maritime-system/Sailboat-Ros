#!/usr/bin/env python
import serial
import time
from math import pi
import sys
import logging

import rospy
from sailboat_message.srv import Dxl_Control_srv
from sailboat_message.srv import Dxl_State_srv
from sailboat_message.srv import Dxl_Control_srvResponse
from sailboat_message.srv import Dxl_State_srvResponse


dynamixel_port = '/dev/dynamixel'


def DXL_MAKEWORD(a, b):
    return (a & 0xFF) | ((b & 0xFF) << 8)

def DXL_MAKEDWORD(a, b):
    return (a & 0xFFFF) | (b & 0xFFFF) << 16

def DXL_LOWORD(l):
    return l & 0xFFFF

def DXL_HIWORD(l):
    return (l >> 16) & 0xFFFF

def DXL_LOBYTE(w):
    return w & 0xFF

def DXL_HIBYTE(w):
    return (w >> 8) & 0xFF

def rad2raw(rad):
    raw = round((rad*4095)/(2*pi))
    if raw <= 0:
        return 0
    elif raw >= 4095:
        return 4095
    else:
        return int(raw)

def raw2rad(raw):
    rad = (raw*2*pi)/4095
    return rad

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


class dxl_driver:
    def __init__(self,num):
        self.logger = console_logger('dxl')
        self.ser_open_flag = self.open()
        self.num = num
        if self.ser_open_flag:
            print ("open_serial")
            for i in range(self.num):
                self.INS_torque_enable(i+1)
        
    def open(self):
        try:
            self.ser = serial.Serial(port=dynamixel_port,baudrate=57600,bytesize=serial.EIGHTBITS,timeout=0)
            time.sleep(2)
            return True
        except(serial.serialutil.SerialException):
            self.logger.info('could not open port: '+dynamixel_port)
            #raise

    def close(self):
        if self.ser_open_flag:
            for i in range(self.num):
                self.INS_torque_disable(i+1)
            self.ser.close()

    def INS_torque_enable(self, dxl_id):
        txpacket = [0] * 8
        txpacket[0] = 0xFF
        txpacket[1] = 0xFF
        txpacket[2] = dxl_id
        txpacket[3] = 4
        txpacket[4] = 3
        txpacket[5] = 24
        txpacket[6] = 1
        checksum = 0
        for idx in range(2, 7):
            checksum += txpacket[idx]
        txpacket[7] = ~checksum & 0xFF
        self.ser.flush()
        self.ser.write(txpacket)

    def INS_torque_disable(self, dxl_id):
        txpacket = [0] * 8
        txpacket[0] = 0xFF
        txpacket[1] = 0xFF
        txpacket[2] = dxl_id
        txpacket[3] = 4
        txpacket[4] = 3
        txpacket[5] = 24
        txpacket[6] = 0
        checksum = 0
        for idx in range(2, 7):
            checksum += txpacket[idx]
        txpacket[7] = ~checksum & 0xFF
        self.ser.flush()
        self.ser.write(txpacket)

    def INS_write_goal_position(self, rad, dxl_id):
        raw = rad2raw(rad)
        txpacket = [0] * 11
        txpacket[0] = 0xFF
        txpacket[1] = 0xFF
        txpacket[2] = dxl_id
        txpacket[3] = 7
        txpacket[4] = 3
        txpacket[5] = 30
        goal_position = [DXL_LOBYTE(DXL_LOWORD(raw)), DXL_HIBYTE(DXL_LOWORD(raw)), DXL_LOBYTE(DXL_HIWORD(raw)), DXL_HIBYTE(DXL_HIWORD(raw))]
        txpacket[6: 10] = goal_position[0: 4]
        checksum = 0
        total_packet_length = 11
        for idx in range(2, 10):
            checksum += txpacket[idx]
        txpacket[10] = ~checksum & 0xFF
        self.ser.flush()
        self.ser.write(txpacket)

    def INS_read_present_position(self, dxl_id):
        txpacket = [0] * 8
        data = []
        txpacket[0] = 0xFF
        txpacket[1] = 0xFF
        txpacket[2] = dxl_id
        txpacket[3] = 4
        txpacket[4] = 2
        txpacket[5] = 36
        txpacket[6] = 4
        checksum = 0
        for idx in range(2, 7):
            checksum += txpacket[idx]
        txpacket[7] = ~checksum & 0xFF
        self.ser.flush()
        self.ser.write(txpacket)
        
    def STATUS_read_present_position(self, dxl_id):
        data = []
        rxpacket = []
        self.INS_read_present_position(dxl_id)
        time.sleep(0.05)
        rxpacket.extend([ord(ch) for ch in self.ser.read(self.ser.inWaiting())])
        rx_length = len(rxpacket)
        print(rxpacket)
        for idx in range(0,(rx_length-1)):
            if (rxpacket[idx] == 0xFF) and (rxpacket[idx+1] == 0xFF) and (rxpacket[idx+2] == dxl_id) and (rxpacket[idx+3] == 6):
                data.extend(rxpacket[idx+5:idx+9])
        present_position = DXL_MAKEDWORD(DXL_MAKEWORD(data[0], data[1]), DXL_MAKEWORD(data[2], data[3]))
        present_position_rad = raw2rad(present_position)
        return present_position_rad


class dxl_driver_ros:
    def __init__(self):
        try:
            sys.exitfunc=self.exitfunc
            rospy.init_node('dxl_driver')
            self.dxl_contrl_srv = rospy.Service('dxl_control_srv',Dxl_Control_srv,self.handle_dxl_control)
            self.dxl_state_srv = rospy.Service('dxl_state_srv',Dxl_State_srv,self.handle_dxl_state)
            self.dxl = dxl_driver(2)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def exitfunc(self):
        self.dxl.close()
        print 'exit is done!'
    
    def handle_dxl_control(self, req):
        dxl_id = req.dxl_id
        position = req.position
        print ('control')
        if self.dxl.ser_open_flag:
            self.dxl.INS_write_goal_position(position,dxl_id)
            result = True
        else:
            result = False
        return Dxl_Control_srvResponse(result)

    def handle_dxl_state(self, req):
        dxl_id = req.dxl_id
        print ('read')
        if self.dxl.ser_open_flag:
            present_position = self.dxl.STATUS_read_present_position(dxl_id)
        else:
            present_position = 9999
        return Dxl_State_srvResponse(present_position)

def run():
    dxl_driver_ros()
    rospy.spin()

if __name__ == '__main__':
    run()








