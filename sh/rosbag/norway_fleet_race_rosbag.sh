#!/usr/bin/env bash

mkdir ~/norway/fleet_race_0905_2_ok

rostopic echo -b /home/hywel/norway/fleet_race_concat_0905_2.bag -p /ahrs            > ~/norway/fleet_race_0905_2_ok/ahrs.txt
rostopic echo -b /home/hywel/norway/fleet_race_concat_0905_2.bag -p /arduino         > ~/norway/fleet_race_0905_2_ok/arduino.txt
rostopic echo -b /home/hywel/norway/fleet_race_concat_0905_2.bag -p /wtst            > ~/norway/fleet_race_0905_2_ok/wtst.txt
rostopic echo -b /home/hywel/norway/fleet_race_concat_0905_2.bag -p /sensor          > ~/norway/fleet_race_0905_2_ok/sensor.txt
rostopic echo -b /home/hywel/norway/fleet_race_concat_0905_2.bag -p /sensor2         > ~/norway/fleet_race_0905_2_ok/sensor2.txt
rostopic echo -b /home/hywel/norway/fleet_race_concat_0905_2.bag -p /mach            > ~/norway/fleet_race_0905_2_ok/mach.txt
rostopic echo -b /home/hywel/norway/fleet_race_concat_0905_2.bag -p /fleet_race_out  > ~/norway/fleet_race_0905_2_ok/fleet_race_out.txt
rostopic echo -b /home/hywel/norway/fleet_race_concat_0905_2.bag -p /fleet_race_para > ~/norway/fleet_race_0905_2_ok/fleet_race_para.txt
