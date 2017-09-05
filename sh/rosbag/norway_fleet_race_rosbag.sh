#!/usr/bin/env bash

mkdir ~/norway/fleet_race_0904

rostopic echo -b ~/norway/fleet_concat.bag -p /ahrs          > ~/norway/fleet_race_0904/ahrs.txt
rostopic echo -b ~/norway/fleet_concat.bag -p /arduino       > ~/norway/fleet_race_0904/arduino.txt
rostopic echo -b ~/norway/fleet_concat.bag -p /wtst          > ~/norway/fleet_race_0904/wtst.txt
rostopic echo -b ~/norway/fleet_concat.bag -p /sensor        > ~/norway/fleet_race_0904/sensor.txt
rostopic echo -b ~/norway/fleet_concat.bag -p /sensor2       > ~/norway/fleet_race_0904/sensor2.txt
rostopic echo -b ~/norway/fleet_concat.bag -p /mach          > ~/norway/fleet_race_0904/mach.txt
rostopic echo -b ~/norway/fleet_concat.bag -p /fleet_race_out  > ~/norway/fleet_race_0904/fleet_race_out.txt
rostopic echo -b ~/norway/fleet_concat.bag -p /fleet_race_para > ~/norway/fleet_race_0904/fleet_race_para.txt
