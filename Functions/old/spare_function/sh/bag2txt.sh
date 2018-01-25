#!/usr/bin/env bash

mkdir ~/rosbag/2017-10-08/spare_function_2017-10-08-11-40-16

rostopic echo -b ~/rosbag/2017-10-08/spare_function_2017-10-08-11-40-16.bag -p /arduino             > ~/rosbag/2017-08-24/scanning_2017_08_24_11_40_16/arduino.txt
rostopic echo -b ~/rosbag/2017-10-08/spare_function_2017-10-08-11-40-16.bag -p /sensor              > ~/rosbag/2017-08-24/scanning_2017_08_24_11_40_16/sensor.txt
rostopic echo -b ~/rosbag/2017-10-08/spare_function_2017-10-08-11-40-16.bag -p /mach                > ~/rosbag/2017-08-24/scanning_2017_08_24_11_40_16/mach.txt
rostopic echo -b ~/rosbag/2017-10-08/spare_function_2017-10-08-11-40-16.bag -p /spare_function_out  > ~/rosbag/2017-08-24/scanning_2017_08_24_11_40_16/spare_function_out.txt
rostopic echo -b ~/rosbag/2017-10-08/spare_function_2017-10-08-11-40-16.bag -p /spare_function_para > ~/rosbag/2017-08-24/scanning_2017_08_24_11_40_16/spare_function_para.txt
