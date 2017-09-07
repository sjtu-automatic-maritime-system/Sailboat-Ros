#!/usr/bin/env bash

mkdir ~/norway/scanning_0906_ok

rostopic echo -b /home/hywel/norway/scanning_0906_concat.bag -p /ahrs            > ~/norway/scanning_0906_ok/ahrs.txt
rostopic echo -b /home/hywel/norway/scanning_0906_concat.bag -p /arduino         > ~/norway/scanning_0906_ok/arduino.txt
rostopic echo -b /home/hywel/norway/scanning_0906_concat.bag -p /wtst            > ~/norway/scanning_0906_ok/wtst.txt
rostopic echo -b /home/hywel/norway/scanning_0906_concat.bag -p /sensor          > ~/norway/scanning_0906_ok/sensor.txt
rostopic echo -b /home/hywel/norway/scanning_0906_concat.bag -p /sensor2         > ~/norway/scanning_0906_ok/sensor2.txt
rostopic echo -b /home/hywel/norway/scanning_0906_concat.bag -p /mach            > ~/norway/scanning_0906_ok/mach.txt
rostopic echo -b /home/hywel/norway/scanning_0906_concat.bag -p /scanning_out    > ~/norway/scanning_0906_ok/scanning_out.txt
rostopic echo -b /home/hywel/norway/scanning_0906_concat.bag -p /scanning_para   > ~/norway/scanning_0906_ok/scanning_para.txt
