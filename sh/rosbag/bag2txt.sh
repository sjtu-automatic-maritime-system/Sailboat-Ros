#!/usr/bin/env bash

mkdir ~/rosbag/2017-08-24/scanning_2017-08-24-11-40-16.bag
mkdir ~/rosbag/2017-08-24/scanning_2017-08-24-13-40-39.bag
mkdir ~/rosbag/2017-08-24/scanning_2017-08-24-11-04-37.bag
mkdir ~/rosbag/2017-08-24/scanning_2017-08-24-14-14-38.bag
mkdir ~/rosbag/2017-08-24/scanning_2017-08-24-11-23-54.bag
mkdir ~/rosbag/2017-08-24/fleet_race_2017-08-24-11-58-27.bag
mkdir ~/rosbag/2017-08-24/station_keeping_2017-08-24-12-12-22.bag

rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-40-16.bag -p /ahrs          > ~/rosbag/2017-08-24/scanning_2017_08_24_11_40_16/ahrs.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-40-16.bag -p /arduino       > ~/rosbag/2017-08-24/scanning_2017_08_24_11_40_16/arduino.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-40-16.bag -p /wtst          > ~/rosbag/2017-08-24/scanning_2017_08_24_11_40_16/wtst.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-40-16.bag -p /sensor        > ~/rosbag/2017-08-24/scanning_2017_08_24_11_40_16/sensor.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-40-16.bag -p /sensor2       > ~/rosbag/2017-08-24/scanning_2017_08_24_11_40_16/sensor2.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-40-16.bag -p /mach          > ~/rosbag/2017-08-24/scanning_2017_08_24_11_40_16/mach.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-40-16.bag -p /scanning_out  > ~/rosbag/2017-08-24/scanning_2017_08_24_11_40_16/scanning_out.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-40-16.bag -p /scanning_para > ~/rosbag/2017-08-24/scanning_2017_08_24_11_40_16/scanning_para.txt

rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-13-40-39.bag -p /ahrs          > ~/rosbag/2017-08-24/scanning_2017_08_24_13_40_39/ahrs.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-13-40-39.bag -p /arduino       > ~/rosbag/2017-08-24/scanning_2017_08_24_13_40_39/arduino.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-13-40-39.bag -p /wtst          > ~/rosbag/2017-08-24/scanning_2017_08_24_13_40_39/wtst.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-13-40-39.bag -p /sensor        > ~/rosbag/2017-08-24/scanning_2017_08_24_13_40_39/sensor.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-13-40-39.bag -p /sensor2       > ~/rosbag/2017-08-24/scanning_2017_08_24_13_40_39/sensor2.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-13-40-39.bag -p /mach          > ~/rosbag/2017-08-24/scanning_2017_08_24_13_40_39/mach.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-13-40-39.bag -p /scanning_out  > ~/rosbag/2017-08-24/scanning_2017_08_24_13_40_39/scanning_out.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-13-40-39.bag -p /scanning_para > ~/rosbag/2017-08-24/scanning_2017_08_24_13_40_39/scanning_para.txt

rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-04-37.bag -p /ahrs          > ~/rosbag/2017-08-24/scanning_2017_08_24_11_04_37/ahrs.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-04-37.bag -p /arduino       > ~/rosbag/2017-08-24/scanning_2017_08_24_11_04_37/arduino.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-04-37.bag -p /wtst          > ~/rosbag/2017-08-24/scanning_2017_08_24_11_04_37/wtst.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-04-37.bag -p /sensor        > ~/rosbag/2017-08-24/scanning_2017_08_24_11_04_37/sensor.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-04-37.bag -p /sensor2       > ~/rosbag/2017-08-24/scanning_2017_08_24_11_04_37/sensor2.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-04-37.bag -p /mach          > ~/rosbag/2017-08-24/scanning_2017_08_24_11_04_37/mach.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-04-37.bag -p /scanning_out  > ~/rosbag/2017-08-24/scanning_2017_08_24_11_04_37/scanning_out.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-04-37.bag -p /scanning_para > ~/rosbag/2017-08-24/scanning_2017_08_24_11_04_37/scanning_para.txt

rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-14-14-38.bag -p /ahrs          > ~/rosbag/2017-08-24/scanning_2017-08_24_14_14_38/ahrs.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-14-14-38.bag -p /arduino       > ~/rosbag/2017-08-24/scanning_2017-08_24_14_14_38/arduino.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-14-14-38.bag -p /wtst          > ~/rosbag/2017-08-24/scanning_2017-08_24_14_14_38/wtst.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-14-14-38.bag -p /sensor        > ~/rosbag/2017-08-24/scanning_2017-08_24_14_14_38/sensor.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-14-14-38.bag -p /sensor2       > ~/rosbag/2017-08-24/scanning_2017-08_24_14_14_38/sensor2.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-14-14-38.bag -p /mach          > ~/rosbag/2017-08-24/scanning_2017-08_24_14_14_38/mach.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-14-14-38.bag -p /scanning_out  > ~/rosbag/2017-08-24/scanning_2017-08_24_14_14_38/scanning_out.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-14-14-38.bag -p /scanning_para > ~/rosbag/2017-08-24/scanning_2017-08_24_14_14_38/scanning_para.txt

rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-23-54.bag -p /ahrs          > ~/rosbag/2017-08-24/scanning_2017_08_24_11_23_54/ahrs.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-23-54.bag -p /arduino       > ~/rosbag/2017-08-24/scanning_2017_08_24_11_23_54/arduino.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-23-54.bag -p /wtst          > ~/rosbag/2017-08-24/scanning_2017_08_24_11_23_54/wtst.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-23-54.bag -p /sensor        > ~/rosbag/2017-08-24/scanning_2017_08_24_11_23_54/sensor.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-23-54.bag -p /sensor2       > ~/rosbag/2017-08-24/scanning_2017_08_24_11_23_54/sensor2.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-23-54.bag -p /mach          > ~/rosbag/2017-08-24/scanning_2017_08_24_11_23_54/mach.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-23-54.bag -p /scanning_out  > ~/rosbag/2017-08-24/scanning_2017_08_24_11_23_54/scanning_out.txt
rostopic echo -b ~/rosbag/2017-08-24/scanning_2017-08-24-11-23-54.bag -p /scanning_para > ~/rosbag/2017-08-24/scanning_2017_08_24_11_23_54/scanning_para.txt

rostopic echo -b ~/rosbag/2017-08-24/fleet_race_2017-08-24-11-58-27.bag -p /ahrs          > ~/rosbag/2017-08-24/fleet_race_2017_08_24_11_58_27/ahrs.txt
rostopic echo -b ~/rosbag/2017-08-24/fleet_race_2017-08-24-11-58-27.bag -p /arduino       > ~/rosbag/2017-08-24/fleet_race_2017_08_24_11_58_27/arduino.txt
rostopic echo -b ~/rosbag/2017-08-24/fleet_race_2017-08-24-11-58-27.bag -p /wtst          > ~/rosbag/2017-08-24/fleet_race_2017_08_24_11_58_27/wtst.txt
rostopic echo -b ~/rosbag/2017-08-24/fleet_race_2017-08-24-11-58-27.bag -p /sensor        > ~/rosbag/2017-08-24/fleet_race_2017_08_24_11_58_27/sensor.txt
rostopic echo -b ~/rosbag/2017-08-24/fleet_race_2017-08-24-11-58-27.bag -p /sensor2       > ~/rosbag/2017-08-24/fleet_race_2017_08_24_11_58_27/sensor2.txt
rostopic echo -b ~/rosbag/2017-08-24/fleet_race_2017-08-24-11-58-27.bag -p /mach          > ~/rosbag/2017-08-24/fleet_race_2017_08_24_11_58_27/mach.txt
rostopic echo -b ~/rosbag/2017-08-24/fleet_race_2017-08-24-11-58-27.bag -p /scanning_out  > ~/rosbag/2017-08-24/fleet_race_2017_08_24_11_58_27/scanning_out.txt
rostopic echo -b ~/rosbag/2017-08-24/fleet_race_2017-08-24-11-58-27.bag -p /scanning_para > ~/rosbag/2017-08-24/fleet_race_2017_08_24_11_58_27/scanning_para.txt

rostopic echo -b ~/rosbag/2017-08-24/station_keeping_2017-08-24-12-12-22.bag -p /ahrs          > ~/rosbag/2017-08-24/station_keeping_2017_08_24_12_12_22/ahrs.txt
rostopic echo -b ~/rosbag/2017-08-24/station_keeping_2017-08-24-12-12-22.bag -p /arduino       > ~/rosbag/2017-08-24/station_keeping_2017_08_24_12_12_22/arduino.txt
rostopic echo -b ~/rosbag/2017-08-24/station_keeping_2017-08-24-12-12-22.bag -p /wtst          > ~/rosbag/2017-08-24/station_keeping_2017_08_24_12_12_22/wtst.txt
rostopic echo -b ~/rosbag/2017-08-24/station_keeping_2017-08-24-12-12-22.bag -p /sensor        > ~/rosbag/2017-08-24/station_keeping_2017_08_24_12_12_22/sensor.txt
rostopic echo -b ~/rosbag/2017-08-24/station_keeping_2017-08-24-12-12-22.bag -p /sensor2       > ~/rosbag/2017-08-24/station_keeping_2017_08_24_12_12_22/sensor2.txt
rostopic echo -b ~/rosbag/2017-08-24/station_keeping_2017-08-24-12-12-22.bag -p /mach          > ~/rosbag/2017-08-24/station_keeping_2017_08_24_12_12_22/mach.txt
rostopic echo -b ~/rosbag/2017-08-24/station_keeping_2017-08-24-12-12-22.bag -p /scanning_out  > ~/rosbag/2017-08-24/station_keeping_2017_08_24_12_12_22/scanning_out.txt
rostopic echo -b ~/rosbag/2017-08-24/station_keeping_2017-08-24-12-12-22.bag -p /scanning_para > ~/rosbag/2017-08-24/station_keeping_2017_08_24_12_12_22/scanning_para.txt
