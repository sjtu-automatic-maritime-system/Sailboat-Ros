#!/usr/bin/env bash

mkdir ~/norway/fleet_race_0905_ok

rostopic echo -b ~/norway/fleet_race_concat_0905.bag -p /ahrs            > ~/norway/fleet_race_0905_ok/ahrs.txt
rostopic echo -b ~/norway/fleet_race_concat_0905.bag -p /arduino         > ~/norway/fleet_race_0905_ok/arduino.txt
rostopic echo -b ~/norway/fleet_race_concat_0905.bag -p /wtst            > ~/norway/fleet_race_0905_ok/wtst.txt
rostopic echo -b ~/norway/fleet_race_concat_0905.bag -p /sensor          > ~/norway/fleet_race_0905_ok/sensor.txt
rostopic echo -b ~/norway/fleet_race_concat_0905.bag -p /sensor2         > ~/norway/fleet_race_0905_ok/sensor2.txt
rostopic echo -b ~/norway/fleet_race_concat_0905.bag -p /mach            > ~/norway/fleet_race_0905_ok/mach.txt
rostopic echo -b ~/norway/fleet_race_concat_0905.bag -p /fleet_race_out  > ~/norway/fleet_race_0905_ok/fleet_race_out.txt
rostopic echo -b ~/norway/fleet_race_concat_0905.bag -p /fleet_race_para > ~/norway/fleet_race_0905_ok/fleet_race_para.txt
