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

class dxl_driver:
    def __init__(self, num=2, timeout_unit=0.005):
        self.logger = self.console_logger('dxl')
        self.ser_open_flag = self.open()
        self.num = num
        self.timeout_unit = timeout_unit
        if self.ser_open_flag:
            print ("open_serial")
            for i in range(self.num):
                self.write_torque_enable(i+1)

    def console_logger(self, name):
        logger = logging.getLogger(name)
        logger.setLevel(logging.INFO)
        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        ch.setFormatter(formatter)
        logger.addHandler(ch)
        return logger

    def open(self):
        try:
            self.ser = serial.Serial(port=dynamixel_port,baudrate=57600,bytesize=serial.EIGHTBITS,timeout=0)
            time.sleep(2)
            return True
        except(serial.serialutil.SerialException):
            self.logger.info('Could not open port: '+dynamixel_port)

    def close(self):
        if self.ser_open_flag:
            for i in range(self.num):
                self.write_torque_disable(i+1)
            self.ser.close()

    def rad2raw(self, rad):
        raw = round(((rad+pi)*4095)/(2*pi))
        if raw <= 0:
            return 0
        elif raw >= 4095:
            return 4095
        else:
            return int(raw)

    def raw2rad(self, raw):
        rad = (raw*2*pi)/4095 - pi
        return rad

    def write_torque_enable(self, dxl_id):
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
        while self.ser.inWaiting() != 6:
            time.sleep(self.timeout_unit)
        self.ser.read(self.ser.inWaiting())

    def write_torque_disable(self, dxl_id):
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
        while self.ser.inWaiting() != 6:
            time.sleep(self.timeout_unit)
        self.ser.read(self.ser.inWaiting())

    def write_goal_position(self, rad, dxl_id):
        raw = self.rad2raw(rad)
        txpacket = [0] * 9
        txpacket[0] = 0xFF
        txpacket[1] = 0xFF
        txpacket[2] = dxl_id
        txpacket[3] = 5
        txpacket[4] = 3
        txpacket[5] = 30
        txpacket[6: 8] = [(raw-((raw >> 8) << 8)), (raw >> 8)]
        checksum = 0
        for idx in range(2, 8):
            checksum += txpacket[idx]
        txpacket[8] = ~checksum & 0xFF
        self.ser.flush()
        self.ser.write(txpacket)
        while self.ser.inWaiting() != 6:
            time.sleep(self.timeout_unit)
        self.ser.read(self.ser.inWaiting())

    def read_present_temperature(self, dxl_id):
        txpacket = [0] * 8
        txpacket[0] = 0xFF
        txpacket[1] = 0xFF
        txpacket[2] = dxl_id
        txpacket[3] = 4
        txpacket[4] = 2
        txpacket[5] = 43
        txpacket[6] = 1
        checksum = 0
        for idx in range(2, 7):
            checksum += txpacket[idx]
        txpacket[7] = ~checksum & 0xFF
        self.ser.flush()	
        self.ser.write(txpacket)
        while self.ser.inWaiting() != 7:
            time.sleep(self.timeout_unit) 
        present_temperature = self.ser.read(self.ser.inWaiting())[-2]
        return present_temperature

    def read_present_position(self, dxl_id):
        txpacket = [0] * 8
        txpacket[0] = 0xFF
        txpacket[1] = 0xFF
        txpacket[2] = dxl_id
        txpacket[3] = 4
        txpacket[4] = 2
        txpacket[5] = 36
        txpacket[6] = 2
        checksum = 0
        for idx in range(2, 7):
            checksum += txpacket[idx]
        txpacket[7] = ~checksum & 0xFF
        self.ser.flush()
        self.ser.write(txpacket)
        while self.ser.inWaiting() != 8:
            time.sleep(self.timeout_unit)
        rxpacket = self.ser.read(self.ser.inWaiting())
        present_position = self.raw2rad(((rxpacket[-2] << 8)+rxpacket[-3]))
        return present_position

class dxl_driver_ros:
    def __init__(self):
        try:
            sys.exitfunc=self.exitfunc
            rospy.init_node('dxl_driver')
            self.dxl_contrl_srv = rospy.Service('dxl_control_srv',Dxl_Control_srv,self.handle_dxl_control)
            self.dxl_state_srv = rospy.Service('dxl_state_srv',Dxl_State_srv,self.handle_dxl_state)
            self.dxl = dxl_driver(num=2)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def exitfunc(self):
        self.dxl.close()
        print 'exit is done!'

    def handle_dxl_control(self, req):
        dxl_id = req.dxl_id
        goal_position = req.position
        print ('control')
        if self.dxl.ser_open_flag:
            self.dxl.write_goal_position(goal_position,dxl_id)
            result = True
        else:
            result = False
        return Dxl_Control_srvResponse(result)

    def handle_dxl_state(self, req):
        dxl_id = req.dxl_id
        print ('read')
        if self.dxl.ser_open_flag:
        	present_temperature = self.dxl.read_present_temperature(dxl_id)
            present_position = self.dxl.read_present_position(dxl_id)
        else:
        	present_temperature = 9999
            present_position = 9999
        return Dxl_State_srvResponse(present_temperature, present_position)

def run():
    dxl_driver_ros()
    rospy.spin()

if __name__ == '__main__':
    run()















