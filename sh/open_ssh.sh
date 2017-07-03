#!/bin/bash

echo "load ros user settings..."

echo "run..."
gnome-terminal --tab --title "t1" -e "sh -c 'ssh sjtu-sailboat@192.168.1.151'" \
--tab --title "t2" -e "sh -c 'ssh sjtu-sailboat@192.168.1.151'" \
# --tab --title "t3" -e "sh -c 'ssh sjtu@192.168.1.101'" \
# --tab --title "t4" -e "sh -c 'ssh sjtu@192.168.1.101'" \

