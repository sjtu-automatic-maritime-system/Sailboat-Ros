#!/usr/bin/env bash

mkdir ~/norway/station_keeping_0905_1_ok

rostopic echo -b /home/hywel/norway/station_keeping_0905/station_keeping_0905_concat_1.bag -p /ahrs                   > ~/norway/station_keeping_0905_1_ok/ahrs.txt
rostopic echo -b /home/hywel/norway/station_keeping_0905/station_keeping_0905_concat_1.bag -p /arduino                > ~/norway/station_keeping_0905_1_ok/arduino.txt
rostopic echo -b /home/hywel/norway/station_keeping_0905/station_keeping_0905_concat_1.bag -p /wtst                   > ~/norway/station_keeping_0905_1_ok/wtst.txt
rostopic echo -b /home/hywel/norway/station_keeping_0905/station_keeping_0905_concat_1.bag -p /sensor                 > ~/norway/station_keeping_0905_1_ok/sensor.txt
rostopic echo -b /home/hywel/norway/station_keeping_0905/station_keeping_0905_concat_1.bag -p /sensor2                > ~/norway/station_keeping_0905_1_ok/sensor2.txt
rostopic echo -b /home/hywel/norway/station_keeping_0905/station_keeping_0905_concat_1.bag -p /mach                   > ~/norway/station_keeping_0905_1_ok/mach.txt
rostopic echo -b /home/hywel/norway/station_keeping_0905/station_keeping_0905_concat_1.bag -p /station_keeping_out    > ~/norway/station_keeping_0905_1_ok/station_keeping_out.txt
rostopic echo -b /home/hywel/norway/station_keeping_0905/station_keeping_0905_concat_1.bag -p /station_keeping_para   > ~/norway/station_keeping_0905_1_ok/station_keeping_para.txt

mkdir ~/norway/station_keeping_0905_2_ok

rostopic echo -b /home/hywel/norway/station_keeping_0905/station_keeping_0905_concat_2.bag -p /ahrs                   > ~/norway/station_keeping_0905_2_ok/ahrs.txt
rostopic echo -b /home/hywel/norway/station_keeping_0905/station_keeping_0905_concat_2.bag -p /arduino                > ~/norway/station_keeping_0905_2_ok/arduino.txt
rostopic echo -b /home/hywel/norway/station_keeping_0905/station_keeping_0905_concat_2.bag -p /wtst                   > ~/norway/station_keeping_0905_2_ok/wtst.txt
rostopic echo -b /home/hywel/norway/station_keeping_0905/station_keeping_0905_concat_2.bag -p /sensor                 > ~/norway/station_keeping_0905_2_ok/sensor.txt
rostopic echo -b /home/hywel/norway/station_keeping_0905/station_keeping_0905_concat_2.bag -p /sensor2                > ~/norway/station_keeping_0905_2_ok/sensor2.txt
rostopic echo -b /home/hywel/norway/station_keeping_0905/station_keeping_0905_concat_2.bag -p /mach                   > ~/norway/station_keeping_0905_2_ok/mach.txt
rostopic echo -b /home/hywel/norway/station_keeping_0905/station_keeping_0905_concat_2.bag -p /station_keeping_out    > ~/norway/station_keeping_0905_2_ok/station_keeping_out.txt
rostopic echo -b /home/hywel/norway/station_keeping_0905/station_keeping_0905_concat_2.bag -p /station_keeping_para   > ~/norway/station_keeping_0905_2_ok/station_keeping_para.txt
