#!/usr/bin/python
# -*- coding: utf-8 -*-

from math import *

PI = 3.1415926

class PlaneCoordinate:

# 平面坐标系
    def __init__(self):
        self.MACRO_AXIS = 6378137.0 # 赤道圆的平均半径
        self.MINOR_AXIS = 6356752.0 # 半短轴的长度，地球两极距离的一半

    def set_basepoint(self, base_latitude, base_longitude):
        self.base_latitude = base_latitude
        self.base_longitude = base_longitude


# 返回Y坐标
    def turnY(self, latitude):
        a = pow(self.MACRO_AXIS, 2.0)
        b = pow(self.MINOR_AXIS, 2.0)
        c = pow(tan(self.base_latitude), 2.0)
        d = pow(1/tan(self.base_latitude),2.0)
        x = a/sqrt(a + b*c)
        y = b/sqrt(b + a*d)

        c = pow(tan(latitude), 2.0)
        d = pow(1/tan(latitude), 2.0)
        
        m = a/sqrt(a + b*c)
        n = b/sqrt(b + a*d)
        
        distance = sqrt(pow(x-m,2.0)+pow(y-n,2.0))
        
        return distance

# 返回X坐标
    def turnX(self, longitude):
        a = pow(self.MACRO_AXIS, 2.0)
        b = pow(self.MINOR_AXIS, 2.0)
        c = pow(tan(self.base_latitude), 2.0)
        x = a/sqrt(a + b*c)

        distance = x * (longitude - self.base_longitude)
        
        return distance


