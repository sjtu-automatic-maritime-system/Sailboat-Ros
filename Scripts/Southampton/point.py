#!/usr/bin/python
from math import *

ORIGIN_LAT = 50.81907
ORIGIN_LON = -1.30718

def d2r(d):
    return d/180.0*pi


def w84_calc_ne(lat2, lon2):
    lat1,lon1 = d2r(ORIGIN_LAT), d2r(ORIGIN_LON)
    lat2,lon2 = d2r(lat2),d2r(lon2)
    d_lat = lat2-lat1
    d_lon = lon2-lon1

    a = 6378137.0
    e_2 = 6.69437999014e-3
    r1 = a*(1-e_2)/(1-e_2*(sin(lat1))**2)**1.5
    r2 = a/sqrt(1-e_2*(sin(lat1))**2)

    north = r1*d_lat
    east = r2*cos(lat1)*d_lon
    print ('north',north)
    print ('east',east)
    return north,east

# print ('B')
# w84_calc_ne(50.822000,-1.31188333)
# print('A')
# w84_calc_ne(50.821183,-1.3117666)
# print('C')
# w84_calc_ne(50.821116,-1.3106333)
# w84_calc_ne(50.82083333,	-1.310983333)

# w84_calc_ne(50.8210729998,	-1.31465664793)
# w84_calc_ne(50.8222207292,	-1.31285527881)
# w84_calc_ne(50.8196023064,	-1.31232136805)
# w84_calc_ne(50.82075,	-1.31052)


w84_calc_ne(50.820540964323925,	-1.313608527048126)
w84_calc_ne(50.820720237801176,	-1.3136307313309448)
w84_calc_ne(50.82082570829222,	-1.311508136900023)
w84_calc_ne(50.820646434491884,	-1.3114859407167054)


w84_calc_ne(50.822197912169486,	-1.3131453808363254)
w84_calc_ne(50.822136584453986,	-1.3128785101937752)
w84_calc_ne(50.82086875660889,	-1.3136046333659215)
w84_calc_ne(50.820930082615554,	-1.313871497810471)