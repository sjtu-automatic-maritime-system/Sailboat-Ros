#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
from numpy import sin, cos, tan
import matplotlib.pyplot as plt
#import read_sensor

import rospy
from sailboat_message.msg import Ahrs_msg
from sailboat_message.msg import WTST_msg
from sailboat_message.msg import Sensor_msg


AHRSinput = [0, 0, 0, 0, 0, 0, 0, 0, 0]
WTSTinput = [0, 0, 0, 0, 0, 0, 0, 0, 0]

detalt = 0.1
isinit=False

class SensorFusion(object):
    def __init__(self, y1, U1, y2, U2,y3):
#	卡尔曼滤波器1：姿态滤波
        self.X1 = y1
        self.P1 = 10 ** 5 * np.matrix(np.eye(3))
        self.Q1 = 3 * 10 ** (-6) * np.matrix(np.eye(3))
        self.R1a = 5 * 10 ** (-4) * np.matrix(np.eye(3))
        self.R1w = 5 * 10 ** (-4) * np.matrix(np.eye(3))
        self.U1 = U1
        self.H1 = np.matrix(np.eye(3))
        self.i = 0
        self.iw = 0
        self.ia = 0

        # self.X1all = self.X1
        # self.P1yaw = [self.P1[2, 2]]
        # self.yf = self.y1

#	卡尔曼滤波器2：位置速度滤波
        self.X2 = np.append(y2[0:3], np.zeros((3, 1)), axis=0)
        self.P2 = 10 ** 1 * np.matrix(np.eye(6))
        self.Q2 = np.matrix([[0.1, 0, 0, 0, 0, 0],
                             [0, 0.1, 0, 0, 0, 0],
                             [0, 0, 0.1, 0, 0, 0],
                             [0, 0, 0, 0.01, 0, 0],
                             [0, 0, 0, 0, 0.01, 0],
                             [0, 0, 0, 0, 0, 0.01]])
        self.R2 = np.matrix([[2, 0, 0, 0, 0],
                             [0, 2, 0, 0, 0],
                             [0, 0, 2, 0, 0],
                             [0, 0, 0, 1, 0],
                             [0, 0, 0, 0, 1]])
        self.U2 = U2


        # self.X2all = self.X2
        # self.U2all = self.U2[3:6]
        # self.deatltpos = np.matrix([[0], [0], [0]])

#	卡尔曼滤波器3：风速滤波
        self.X3 = y3
        self.P3 = 10 ** 5 * np.matrix(np.eye(2))
        self.Q3 = 3 * 10 ** (-5) * np.matrix(np.eye(2))
        self.R3 = 3 * 10 ** (-2) * np.matrix(np.eye(2))

        self.X3all = self.X3

    def yawRange(self, x):
        if x > np.pi:
            x = x - 2 * np.pi
        elif x < -np.pi:
            x = x + 2 * np.pi
        return x

    def diff(self, x, y):
        if x - y > np.pi:
            y = y + 2 * np.pi
        elif x - y < -np.pi:
            y = y - 2 * np.pi
        return y

    def yawFusion(self, wtstyaw, ahrsyaw, heading):
        R1w = self.R1w.copy()
        R1a = self.R1a.copy()
        if self.i == 500:
            if self.iw > self.ia:
                R1w[2, 2] = 100 * self.R1w[2, 2]
                print('drop wtst')
            else:
                R1a[2, 2] = 100 * self.R1a[2, 2]
                print('drop ahrs')
            return R1w, R1a
        if abs(ahrsyaw - self.diff(ahrsyaw, wtstyaw)) > 1.0:
            print('ahrs and wtst are much different')
            if np.sqrt(self.X2[3, 0] ** 2 + self.X2[4, 0] ** 2) > 0.2 and self.X2[4, 0] < 0.4:
                self.i = self.i + 1
                if abs(ahrsyaw - self.diff(ahrsyaw, heading)) < abs(wtstyaw - self.diff(wtstyaw, heading)):
                    R1w[2, 2] = 1000 * self.R1w[2, 2]
                    R1a[2, 2] = 100 * self.R1a[2, 2]
                    self.iw = self.iw + 1
                else:
                    R1w[2, 2] = 100 * self.R1w[2, 2]
                    R1a[2, 2] = 1000 * self.R1a[2, 2]
                    self.ia = self.ia + 1
                print(self.i, 'speed angle close to ahrs',self.iw,'to wtst', self.ia)
        return R1w, R1a

    def ObserveFusion(self, y1, y2, heading):
        R1w, R1a = self.yawFusion(y1[2, 0], y2[2, 0], heading)
        R = (R1w.I + R1a.I).I
        y1[2, 0] = self.diff(y2[2, 0], y1[2, 0])
        yf = R * (R1w.I * y1 + R1a.I * y2)
        yf[2, 0] = self.yawRange(yf[2, 0])
        # self.yf = np.append(self.yf, yf, axis=1)
        return yf, R

    def AttitudeUpdate(self, y1, y2, U1, heading):
        roll = self.X1[0, 0]
        pitch = self.X1[1, 0]
        # p = self.U1[0, 0]
        q = self.U1[1, 0]
        r = self.U1[2, 0]

        T = np.matrix([[1, sin(roll) * tan(pitch), cos(roll) * tan(pitch)], [0, cos(pitch), -sin(pitch)],
                       [0, sin(roll) / cos(pitch), cos(roll) / cos(pitch)]])
        A = np.eye(3)
        jacobian = np.matrix([[(q * cos(roll) - r * sin(roll)) * tan(pitch) * detalt + 1,
                               (r * cos(roll) + q * sin(roll)) * (tan(pitch) ** 2 + 1) * detalt, 0],
                              [-(r * cos(roll) + q * sin(roll)) * detalt, 1, 0],
                              [(q * cos(roll) / cos(pitch) - r * sin(roll) / cos(pitch)) * detalt,
                               (q * sin(pitch) * sin(roll) + r * sin(pitch) * cos(roll)) / cos(pitch) ** 2 * detalt,
                               1]])
        yf, R = self.ObserveFusion(y1, y2, heading)
        self.X1, self.P1 = self.EKF(self.X1, yf, A, T * detalt, self.U1, jacobian, self.H1, self.P1, self.Q1, R)
        self.U1 = U1
        # self.X1all = np.append(self.X1all, self.X1, axis=1)


    def EKF(self, x, y, A, B, u, jacobianA, H, P, Q, R):
        xp = A * x + B * u
        Pp = jacobianA * P * jacobianA.T + Q
        k = Pp * H.T * (H * Pp * H.T + R).I
        Hxp = H * xp
        y[2, 0] = self.diff(Hxp[2, 0], y[2, 0])
        x = xp + k * (y - Hxp)
        P = (np.matrix(np.eye(P.shape[0])) - k * H) * Pp
        x[2, 0] = self.yawRange(x[2, 0])
        return x, P

    def PositionUpdate(self, y, U2):
        roll = self.X1[0, 0]
        pitch = self.X1[1, 0]
        yaw = self.X1[2, 0]
        B = np.eye(6)
        Rnb = np.matrix([[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll),
                          sin(yaw) * sin(roll) + cos(yaw) * cos(roll) * sin(pitch)],
                         [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(roll) * sin(pitch) * sin(yaw),
                          -cos(yaw) * sin(roll) + sin(pitch) * sin(yaw) * cos(roll)],
                         [-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(roll)]])
        # self.deatltpos = np.hstack((self.deatltpos, Rnb * self.X2[3:6]))
        self.H2 = np.matrix([[1, 0, 0, 0, 0, 0],
                             [0, 1, 0, 0, 0, 0],
                             [0, 0, 1, 0, 0, 0],
                             [0, 0, 0, cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll),
                              sin(yaw) * sin(roll) + cos(yaw) * cos(roll) * sin(pitch)],
                             [0, 0, 0, sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(roll) * sin(pitch) * sin(yaw),
                              -cos(yaw) * sin(roll) + sin(pitch) * sin(yaw) * cos(roll)]])
        y[0:3] = y[0:3] - Rnb * np.matrix([[-0.61], [0], [-0.8]])
        # self.U2[3:6] = self.U2[3:6] + Rnb.T * np.matrix([[0], [0], [9.8]])
        self.U2 = np.matrix(np.zeros((6, 1)))
        # acc = Rnb * self.U2[3:6]

        A = np.append(np.append(np.eye(3), Rnb * detalt, axis=1), np.append(np.zeros((3, 3)), np.eye(3), axis=1),
                      axis=0)
        self.X2, self.P2 = self.KF(self.X2, y, A, B * detalt, self.U2, self.H2, self.P2, self.Q2, self.R2)
        self.U2 = U2
        # self.X2all = np.append(self.X2all, self.X2, axis=1)
        # self.U2all = np.append(self.U2all, acc, axis=1)

    def Windupdate(self, y):
        roll = self.X1[0, 0]
        pitch = self.X1[1, 0]
        yaw = self.X1[2, 0]
        B = np.eye(2)
        Rnb2 = np.matrix([[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll)],
                          [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(roll) * sin(pitch) * sin(yaw)]])
        H3 = Rnb2.T
        A = np.eye(2)
        U3 = np.matrix(np.zeros((2, 1)))
        y = y - self.X2[3:5, 0]
        self.X3, self.P3 = self.KF(self.X3, y, A, B * detalt, U3, H3, self.P3, self.Q3, self.R3)
        self.X3all = np.append(self.X3all, self.X3, axis=1)


    def KF(self, x, y, A, B, u, H, P, Q, R):
        xp = A * x + B * u
        Pp = A * P * A.T + Q
        k = Pp * H.T * (H * Pp * H.T + R).I
        x = xp + k * (y - H * xp)
        P = (np.matrix(np.eye(P.shape[0])) - k * H) * Pp
        return x, P




def pretreat(wtst):
    wtst[4] = wtst[4] * np.pi / 180
    wtst[5] = (wtst[5]+5.04 )* np.pi / 180
    wtst[6] = wtst[6] * np.pi / 180
    wtst[2] = wtst[2] * np.pi / 180
    if wtst[6]>np.pi:
        wtst[6]=wtst[6]-2*np.pi
    if wtst[2]>np.pi:
        wtst[2]=wtst[2]-2*np.pi
    return wtst



def wtst_callback(data):
    global WTSTinput
    #print ('start')
    #rospy.loginfo("I heard %f", data.roll)
    WTSTinput[0] = data.PosX
    WTSTinput[1] = data.PosY
    WTSTinput[2] = data.DegreeTrue
    WTSTinput[3] = data.SpeedKnots
    WTSTinput[4] = data.Roll
    WTSTinput[5] = data.Pitch
    WTSTinput[6] = data.Yaw
    WTSTinput[7] = data.WindAngle
    WTSTinput[8] = data.WindSpeed
    rospy.loginfo("I heard WTST %f", data.WindAngle)

def ahrs_callback(data):
    global AHRSinput
    AHRSinput[0] = data.roll
    AHRSinput[1] = data.pitch
    AHRSinput[2] = data.yaw
    AHRSinput[3] = data.gx
    AHRSinput[4] = data.gy
    AHRSinput[5] = data.gz
    AHRSinput[6] = data.ax
    AHRSinput[7] = data.ay
    AHRSinput[8] = data.az
    rospy.loginfo("I heard ahrs %f", data.roll)



if __name__ == "__main__":
    # AHRSdata = read_sensor.read_ahrs('./fleet_race_0905_ok/ahrs.txt', (2, 4, 5, 6, 7, 8, 9, 10, 11, 12))  # (time,roll,pitch,yaw,gx,gy,gz,ax,ay,az)
    # WTSTdata = read_sensor.read_wtst('./fleet_race_0905_ok/wtst.txt', (2, 9, 10, 11, 12, 13, 14, 15, 16, 17))  # (time,posx,posy,speedangle,speed,roll,pitch,yaw,windangle,windspeed)
    # AHRSdata = read_sensor.read_ahrs('ahrs.txt',(2, 4, 5, 6, 7, 8, 9, 10, 11, 12))  # (time,roll,pitch,yaw,gx,gy,gz,ax,ay,az)
    # WTSTdata = read_sensor.read_wtst('wtst.txt', (2, 8, 9, 10, 11, 12, 13, 14, 15, 16))  # (time,posx,posy,speedangle,speed,roll,pitch,yaw,windangle,windspeed)
    # AHRSdata, WTSTdata = read_sensor.pre_treat(AHRSdata, WTSTdata)
    # AHRSdata = np.delete(AHRSdata, range(9000), 0)
    # WTSTdata = np.delete(WTSTdata, range(9000), 0)


    #plt.close()  # clf() # 清图  cla() # 清坐标轴 close() # 关窗口
    #fig = plt.figure()
    #ax1 = fig.add_subplot(1, 1, 1)
    # ax2= fig.add_subplot(1, 2, 2)
    #plt.grid(True)  # 添加网格
    #plt.ion()  # interactive mode on
    #print('开始仿真')
    rospy.init_node('sensor_kalman', anonymous=True)
    pub = rospy.Publisher('sensor_kalman_msg', Sensor_msg, queue_size=5)
    rospy.Subscriber('wtst', WTST_msg, wtst_callback)
    rospy.Subscriber('ahrs', Ahrs_msg, ahrs_callback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # WTSTinput [posx,posy,speedangle,speed,roll,pitch,yaw,windangle,windspeed]
        # AHRSinput [roll,pitch,yaw,gx,gy,gz,ax,ay,az]
        # WTSTinput=WTSTdata[i,1:10]
        # AHRSinput=AHRSdata[i,1:10]
        WTSTinput=pretreat(WTSTinput)

        pos=np.matrix([WTSTinput[0],WTSTinput[1],0,WTSTinput[3]*cos(WTSTinput[2]),WTSTinput[3]*sin(WTSTinput[2])]).T
        U2=np.matrix([0,0,0,AHRSinput[6],AHRSinput[7],AHRSinput[8]]).T
        yw=np.matrix([WTSTinput[4],WTSTinput[5],WTSTinput[6]]).T
        ya=np.matrix([AHRSinput[0],AHRSinput[1],AHRSinput[2]]).T
        U1A=np.matrix([AHRSinput[3],AHRSinput[4],AHRSinput[5]]).T
        wind=np.matrix([WTSTinput[8]*cos(WTSTinput[7]),WTSTinput[8]*sin(WTSTinput[7])]).T

        if isinit ==False:
            SF=SensorFusion(yw, U1A, pos, U2,wind)
            isinit=True

        SF.Windupdate(wind)
        SF.PositionUpdate(pos,U2)
        SF.AttitudeUpdate(yw, ya, U1A, WTSTinput[2])

        # ax1.scatter(i,WTSTinput[2], c='b', marker='.',label='DegreeTrue')
        # ax1.scatter(i, WTSTinput[6], c='g', marker='.', label='WTSTyaw')
        # ax1.scatter(i, AHRSinput[2], c='r', marker='.', label='AHRSyaw')
        # ax1.scatter(i, SF.X1[2,0], c='m', marker='.', label='yaw')

        # ax1.scatter(i,np.sqrt(SF.X3[0,0]**2+SF.X3all[1,0]**2),c='b', marker='.',label='windspeed')
        # ax1.scatter(i,np.arctan2(SF.X3[1,0],SF.X3[0,0]),c='r', marker='.',label='windangle')

        # plt.pause(0.001)
        # output [roll,pitch,yaw,N,E,u,v,w,TWA,TWS]
        output=[SF.X1[0,0],SF.X1[1,0],SF.X1[2,0],SF.X2[0,0],SF.X2[1,0],SF.X2[3,0],SF.X2[4,0],SF.X2[5,0]]
        msg = Sensor_msg()
        msg.ux = output[5]
        msg.vy = output[6]
        msg.wz = output[7]
        msg.gx = AHRSinput[3]
        msg.gy = AHRSinput[4]
        msg.gz = AHRSinput[5]
        msg.Posx = output[3]
        msg.Posy = output[4]
        msg.PosZ = 0
        msg.Roll = output[0]
        msg.Pitch = output[1]
        msg.Yaw = output[2]
        msg.AWA = WTSTinput[7]
        msg.AWS = WTSTinput[8]
        msg.TWA = 0 #output[8]
        msg.TWS = 0 #output[9]

        pub.publish(msg)
        rate.sleep()
    # Xspeed = np.hstack((np.zeros(10), WTSTdata[10:length, 1] - WTSTdata[0:length - 10, 1])) / 1
    # thta = np.arctan2(np.array(SF.X2all)[3], np.array(SF.X2all)[4])
    # plt.figure(3)
    # plt.plot(WTSTdata[0:6000, 3], label='DegreeTrue')
    # plt.plot(np.array(SF.yf)[2], label='yf')
    # plt.plot(WTSTdata[0:6000, 7], label='WTSTyaw')
    # plt.plot(AHRSdata[0:6000, 3], label='AHRSyaw')
    # plt.plot(np.array(SF.X1all)[2], label='yaw')
    # plt.legend(loc='upper right')
    # plt.figure(4)
    # plt.plot(WTSTdata[:, 4])
    # plt.plot(np.sqrt(np.array(SF.X2all)[3] ** 2 + np.array(SF.X2all)[4] ** 2))
    # plt.figure(5)
    # plt.plot(np.array(SF.X2all)[0], np.array(SF.X2all)[1], label='x-y')
    # plt.plot(WTSTdata[0:length, 1], WTSTdata[0:length, 2], label='POS')
    # plt.legend(loc='upper right')
    # plt.figure(6)
    # plt.subplot((131))
    # plt.plot(np.array(SF.X2all)[3], label='u')
    # plt.legend(loc='upper right')
    # plt.subplot((132))
    # plt.plot(np.array(SF.X2all)[4], label='v')
    # plt.legend(loc='upper right')
    # plt.subplot((133))
    # plt.plot(np.array(SF.X2all)[5], label='w')
    # plt.legend(loc='upper right')
    # plt.figure(7)
    # # plt.plot(np.array(dX)[0],label='dX')
    # plt.plot(np.array(np.delete(SF.deatltpos, 0, 1))[0], label='deatltpos')
    # plt.plot(Xspeed, label='Xspeed')
    # plt.legend(loc='upper right')
    # plt.show()

