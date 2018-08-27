#!/usr/bin/env python

import serial
from math import pi
import time
import logging
import sys

import rospy
from sailboat_message.srv import Dxl_Control_srv
from sailboat_message.srv import Dxl_State_srv
from sailboat_message.srv import Dxl_Control_srvResponse
from sailboat_message.srv import Dxl_State_srvResponse
import std_srvs

class dxl_driver:
    def __init__(self, dynamixel_port='/dev/dynamixel', dxl_num=2):
        self.logger = self.console_logger('dxl')
        self.ser_open_flag = self.open(dynamixel_port)
        self.dxl_num = dxl_num
        self.dxl_overload = False
        if self.ser_open_flag:
            print "Open serial successfully!"
            for i in range(self.dxl_num):
                self.write_status_return_level(i+1)
                self.read_status_return_level(i+1)
                self.write_torque_enable(i+1,1)
                self.read_torque_enable(i+1)

    def console_logger(self, name):
        logger = logging.getLogger(name)
        logger.setLevel(logging.INFO)
        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        ch.setFormatter(formatter)
        logger.addHandler(ch)
        return logger

    def open(self, dynamixel_port):
        try:
            self.ser = serial.Serial(port=dynamixel_port, baudrate=57600, bytesize=serial.EIGHTBITS, timeout=0.05)
            time.sleep(2)
            return True
        except(serial.serialutil.SerialException):
            self.logger.info('Could not open port: '+dynamixel_port)
            print 'Could not open dynamixel port!'

    def close(self):
        if self.ser_open_flag:
            for i in range(self.dxl_num):
                self.write_torque_enable(i+1,0)
                self.read_torque_enable(i+1)
            self.ser.close()

    def write_status_return_level(self, dxl_id):
        self.ser.flushOutput()
        txpacket = [0] * 8
        txpacket[0] = 0xff
        txpacket[1] = 0xff
        txpacket[2] = dxl_id
        txpacket[3] = 4
        txpacket[4] = 3
        txpacket[5] = 16
        txpacket[6] = 1
        checksum = 0
        for idx in range(2,7):
            checksum += txpacket[idx]
        txpacket[7] = ~checksum & 0xff
        self.ser.flush()
        self.ser.write(txpacket)

    def read_status_return_level(self, dxl_id):
        self.ser.flushInput()
        self.ser.flushOutput()
        txpacket = [0] * 8
        txpacket[0] = 0xff
        txpacket[1] = 0xff
        txpacket[2] = dxl_id
        txpacket[3] = 4
        txpacket[4] = 2
        txpacket[5] = 16
        txpacket[6] = 1
        checksum = 0
        for idx in range(2,7):
            checksum += txpacket[idx]
        txpacket[7] = ~checksum & 0xff
        self.ser.flush()
        self.ser.write(txpacket)
        rxpacket = self.ser.read(7)
        if len(rxpacket) == 7 and ord(rxpacket[4]) == 0:
            if ord(rxpacket[5]) == 1:
                print "[ID: %d] Status return level has been set successfully!" % dxl_id
                return True
            else:
                print "[ID: %D] Status return level is not correct!" % dxl_id
                return False
        else:
            print "Read status return level failed!"
            return False

    def write_torque_enable(self, dxl_id, value):
        self.ser.flushOutput()
        txpacket = [0] * 8
        txpacket[0] = 0xff
        txpacket[1] = 0xff
        txpacket[2] = dxl_id
        txpacket[3] = 4
        txpacket[4] = 3
        txpacket[5] = 24
        txpacket[6] = value
        checksum = 0
        for idx in range(2,7):
            checksum += txpacket[idx]
        txpacket[7] = ~checksum & 0xff
        self.ser.flush()
        self.ser.write(txpacket)

    def read_torque_enable(self, dxl_id):
        self.ser.flushInput()
        self.ser.flushOutput()
        txpacket = [0] * 8
        txpacket[0] = 0xff
        txpacket[1] = 0xff
        txpacket[2] = dxl_id
        txpacket[3] = 4
        txpacket[4] = 2
        txpacket[5] = 24
        txpacket[6] = 1
        checksum = 0
        for idx in range(2,7):
            checksum += txpacket[idx]
        txpacket[7] = ~checksum & 0xff
        self.ser.flush()
        self.ser.write(txpacket)
        rxpacket = self.ser.read(7)
        if len(rxpacket) == 7 and ord(rxpacket[4]) == 0:
            if ord(rxpacket[5]) == 1:
                print "[ID: %d] Torque enable already!" % dxl_id
                return True
            else:
                print "[ID: %d] Torque disable already!" % dxl_id
                return False
        else:
            print "Read torque enable failed!"
            return False

    def rad2raw(self, rad):
        raw = int(round(((rad+pi)*4095)/(2*pi)))
        if raw <= 0:
            return 0
        elif raw >= 4095:
            return 4095
        else:
            return raw

    def raw2rad(self, raw):
        rad = (raw*2*pi)/4095 - pi
        return rad

    def read_present_temperature(self, dxl_id):
        self.ser.flushInput()
        self.ser.flushOutput()
        txpacket = [0] * 8
        txpacket[0] = 0xff
        txpacket[1] = 0xff
        txpacket[2] = dxl_id
        txpacket[3] = 4
        txpacket[4] = 2
        txpacket[5] = 43
        txpacket[6] = 1
        checksum = 0
        for idx in range(2,7):
            checksum += txpacket[idx]
        txpacket[7] = ~checksum & 0xff
        self.ser.flush()
        self.ser.write(txpacket)
        rxpacket = self.ser.read(7)
        if len(rxpacket) == 7 and ord(rxpacket[4]) == 0:
            return ord(rxpacket[5])
        else:
            return -1000

    def write_goal_position(self, rad, dxl_id):
        self.ser.flushOutput()
        raw = self.rad2raw(rad)
        txpacket = [0] * 9
        txpacket[0] = 0xff
        txpacket[1] = 0xff
        txpacket[2] = dxl_id
        txpacket[3] = 5
        txpacket[4] = 3
        txpacket[5] = 30
        txpacket[6:8] = [(raw-((raw >> 8) << 8)), (raw >> 8)]
        checksum = 0
        for idx in range(2,8):
            checksum += txpacket[idx]
        txpacket[8] = ~checksum & 0xff
        self.ser.flush()
        self.ser.write(txpacket)

    def read_present_position(self, dxl_id):
        self.ser.flushInput()
        self.ser.flushOutput()
        txpacket = [0] * 8
        txpacket[0] = 0xff
        txpacket[1] = 0xff
        txpacket[2] = dxl_id
        txpacket[3] = 4
        txpacket[4] = 2
        txpacket[5] = 36
        txpacket[6] = 2
        checksum = 0
        for idx in range(2,7):
            checksum += txpacket[idx]
        txpacket[7] = ~checksum & 0xff
        self.ser.flush()
        self.ser.write(txpacket)
        rxpacket = self.ser.read(8) 
        if len(rxpacket) == 8 and ord(rxpacket[4]) == 0:
            present_position = self.raw2rad(((ord(rxpacket[-2]) << 8)+ord(rxpacket[-3])))
            return present_position
        elif len(rxpacket) == 8 and ord(rxpacket[4]) == 32:
            self.dxl_overload = True
            return 100000
        else:
            return 1000

    def write_reboot(self):
        for dxl_num in range(self.dxl_num):
            dxl_id = dxl_num + 1
            self.ser.flushOutput()
            txpacket = [0] * 9
            txpacket[0] = 0xff
            txpacket[1] = 0xff
            txpacket[2] = dxl_id
            txpacket[3] = 5
            txpacket[4] = 3
            txpacket[5] = 34
            torque = 1023
            txpacket[6:8] = [(torque-((torque >> 8) << 8)), (torque >> 8)]
            checksum = 0
            for idx in range(2,8):
                checksum += txpacket[idx]
            txpacket[8] = ~checksum & 0xff
            self.ser.flush()
            self.ser.write(txpacket)
        self.dxl_overload = False


class dxl_driver_ros:
    def __init__(self):
        try:
            sys.exitfunc = self.exitfunc
            rospy.init_node('dxl_driver')
            self.dxl = dxl_driver(dynamixel_port='/dev/dynamixel', dxl_num=2)
            self.dxl_control_srv = rospy.Service('dxl_control_srv', Dxl_Control_srv, self.handle_dxl_control)
            self.dxl_state_srv = rospy.Service('dxl_state_srv', Dxl_State_srv, self.handle_dxl_state)
            self.dxl_reboot_srv = rospy.Service('dxl_reboot_srv', Dxl_State_srv, self.handle_dxl_reboot)
        except:
            print 'Service initial failed!'

    def exitfunc(self):
        self.dxl.close()
        print 'exit is done!'

    def handle_dxl_control(self, req):
        dxl_id = req.dxl_id
        position = req.position
        if not self.dxl.dxl_overload:
            if self.dxl.ser_open_flag:
                print 'control'
                self.dxl.write_goal_position(position, dxl_id)
                result = True
            else:
                print 'not connect'
                result = False
        elif self.dxl.dxl_overload:
            print 'overload'
            result = False
        else:
            print 'error'
            result = False
        return Dxl_Control_srvResponse(result)

    def handle_dxl_state(self, req):
        dxl_id = req.dxl_id
        if self.dxl.ser_open_flag:
            print 'state'
            present_position = self.dxl.read_present_position(dxl_id)
        else:
            print 'not connect'
            present_position = 9999
        return Dxl_State_srvResponse(present_position)

    def handle_dxl_reboot(self, req):
        if self.dxl.ser_open_flag:
            print 'reboot'
            self.dxl.write_reboot()
        else:
            print 'not connect'
        return Dxl_State_srvResponse(1)

def run():
    dxl_driver_ros()
    rospy.spin()

if __name__ == '__main__':
    run()

