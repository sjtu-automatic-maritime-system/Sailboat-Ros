#!/usr/bin/env python

import struct
import binascii
import serial
import time
# from msgdev import

arduino_port = '/dev/serial/by-id/usb-Arduino_Srl_Arduino_Mega_855353036363516120C1-if00'

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
        self.recvDataBytes = 14
        self.recvDataFst = struct.Struct('<H5hH')

        self.buf = ''

        self.motor = 0
        self.rudder = 0
        self.sail = 0


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
            self.send_data()

    def read_data(self):
        self.buf += self.arduino_ser.read(14-len(self.buf))
        print binascii.hexlify(self.buf)
        idx = self.buf.find(self.header)
        if idx < 0:
            self.buf = ''
            print ('ReadError: header not found, discard buffer')
            return
        elif idx > 0:
            self.buf = self.buf[idx:]
            print ('ReadError: header not at start, discard bytes before header')
            return
        if len(self.buf) < 14:
            print ('ReadError: not enough data')
            return

        #testBety = self.buf[0:9]

        datas = self.recvDataFst.unpack(self.buf)
        print datas
        crc16Num = crc16(self.buf[2:-2])


        if datas[-1] != crc16Num:
            self.buf = self.buf[2:]
            print ('ReadError: ckcum error, discard first 2 bytes')
            return

        self.EarduinoDatas = datas[1:-1]
        print self.EarduinoDatas

        #print (type(self.buf))
        #print ('bety = ', testBety)
        #print ('Roll = ', datas[0])
        #abc = hex(datas[0])
        #print ('Roll = ', abc)

        self.buf = ''

    def send_data(self):
        header = '\xff\x01'
        tmp = self.fst.pack(self.motor,self.rudder,self.sail,0)
        crc_code = struct.pack('!H', crc16(tmp))
        # print binascii.hexlify(tmp), type(tmp)
        tmp = header + tmp
        tmp = tmp + crc_code
        print binascii.hexlify(tmp)
        return tmp

    def close(self):
        self.arduino_ser.close()





def talker():#ros message publish
    arduino = Arduino()
    try:
        while 1:
            arduino.update()
    finally:
        arduino.close()



if __name__ == '__main__':

    talker()

