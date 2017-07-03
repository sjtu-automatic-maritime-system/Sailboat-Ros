# WeatherStation_Talker.py

## has two message files

### WTST_msg (same with WTST_Pro_msg)
#### int16 GPSIndicator
#### float64 Latitude
#### float64 Longitude
#### float64 PosX
#### float64 PosY
#### float64 Roll
#### float64 Pitch
#### float64 Yaw
#### float64 WindAngle
#### float64 WindSpeed

### WTST_Pro_msg
#### int16 GPSIndicator
###### GPS quality indicator:
###### 0 = Fix not available or invalid          1 = GPS SPS Mode, fix valid
###### 2 = Differential GPS, SPS Mode, fix valid 3 = GPS PPS Mode, fix valid
###### 4 = Real Time Kinematic (RTK)             5 = Float RTK
###### 6 = Estimated (dead reckoning) Mode       7 = Manual Input Mode
###### 8 = Simulator Mode
#### float64 Latitude
###### Latitude, to the nearest .0001 minute ddmm.mmmm
#### float64 Longitude
###### Longitude, to the nearest .0001 minute ddmm.mmmm
#### float64 PosX
###### North
#### float64 PosY
###### East
#### int16 NumSata
###### Number of satellites in use, 0-12
#### string VTGIndicator
###### Mode indicator:
###### A = Autonomous mode    D = Differential mode  E = Estimated (dead reckoning) mode
###### M = Manual input mode  S = Simulator mode     N = Data not valid
#### float64 DegreeTrue
###### Course Over Ground, degrees True, to the nearest 0.1 degree
#### float64 DegreeMagnetic
###### Course Over Ground, degrees Magnetic, to the nearest 0.1 degree
#### float64 SpeedKnots
###### Speed Over Ground, knots, to the nearest 0.1 knot
#### float64 HeadingMagneticSenor
###### Magnetic sensor heading, degrees, to the nearest 0.1 degree
#### float64 MagneticDeviation
###### Magnetic deviation, degrees east or west, to the nearest 0.1 degree.
#### string DirectionDeviation
###### E or W
#### float64 MagneticVariation
###### Magnetic variation, degrees east or west, to the nearest 0.1 degree.
#### string DirectionVariation
###### E or W
#### float64 Roll
###### Roll: oscillation of vessel about its longitudinal axis. Roll to the starboard is positive. Value reported to the nearest 0.1 degree.
#### float64 Pitch
###### Pitch: oscillation of vessel about its latitudinal axis. Bow moving up is positive. Value reported to the nearest 0.1 degree.
#### float64 Yaw
###### Heading relative to True North, degrees
#### float64 BarMercury
###### Barometric pressure, inches of mercury, to the nearest 0.01 inch
#### float64 AirTemperature
###### Air temperature, degrees C, to the nearest 0.1 degree C
#### float64 WindDirectionTrue
###### Wind direction, degrees True, to the nearest 0.1 degree
#### float64 WindDirectionMagnetic
###### Wind direction, degrees Magnetic, to the nearest 0.1 degree
#### float64 WindSpeedKnots
###### Wind speed, knots, to the nearest 0.1 knot
#### string MWVStatus
###### Status: A = data valid; V = data invalid
#### float64 WindAngle
###### Wind angle, 0.0 to 359.9 degrees, in relation to the vessel’s bow/centerline, to the nearest 0.1 degree. If the data for this field is not valid, the field will be blank.
#### float64 WindSpeed
###### Wind speed, to the nearest tenth of a unit. If the data for this field is not valid, the field will be blank.

# Ahrs_Talker.py

### Ahrs_msg
#### int16 AhrsFlag
###### 1 = get data
#### float64 roll
###### Roll angle in degrees [-180; 180]
#### float64 pitch
###### Pitch angle in degrees [-90;90]
#### float64 yaw
###### Yaw angle in degrees [-180; 180]
#### float64 gx
#### float64 gy
#### float64 gz
###### Fully calibrated and Kalman unbiased gyroscopes values. G x , G y and G z are three real32 numbers(12 bytes), expressed in rad . s -1 .
#### float64 wx
#### float64 wy
#### float64 wz
###### Fully calibrated and Kalman unbiased accelerometers values. A x , A y and A z are three real32 numbers (12 bytes), expressed in m . s -2 .
