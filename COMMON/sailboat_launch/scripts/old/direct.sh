#!/bin/bash

delay=$1
program=$2
param1=$3
param2=$4

echo "1: delay: $delay"
echo "2: program: $program"
echo "3: param 1 is: $param1"
echo "4: param 2 is: $param2"
echo "delay $delay"
echo "Start $program $param1 $param2"

if [[ -z "$program" ]]
then
    echo "Wrong parameters!"
    exit 1
fi

self_PID=$$
echo "self_PID=$self_PID"

#rosmaster_PID=`lsof -t -i :11311|head -n 1`


skip_flag=false

if [[ "$program" != "roscore" ]]
then
    #kill the current running program first.
    first_char="${program:0:1}"
    rest_chars="${program:1}"
	if [[ ! -z $param2 ]]
	then
		program_PIDS=$(ps -ef | grep "\\s[$first_char]$rest_chars.*$param2" | grep -v "direct.sh" | awk '{print $2}')
		echo "Got old program process(es):"
		ps -ef | grep "\\s[$first_char]$rest_chars.*$param2" | grep -v "direct.sh"
		terminal_PIDS=$(ps -ef | grep "direct.sh.*[$first_char]$rest_chars.*$param2" | grep -v "$self_PID" | grep -v "xfce4-terminal" | awk '{print $2}')
        echo "Got old terminal process(es):"
        ps -ef | grep "direct.sh.*[$first_char]$rest_chars.*$param2" | grep -v "$self_PID" | grep -v "xfce4-terminal"
	else
		program_PIDS=$(ps -ef | grep "\\s[$first_char]$rest_chars\$" | grep -v "direct.sh" | awk '{print $2}')
		echo "Got old program process(es):"
		ps -ef | grep "\\s[$first_char]$rest_chars\$" | grep -v "direct.sh"
		terminal_PIDS=$(ps -ef | grep "direct.sh.*[$first_char]$rest_chars\$" | grep -v "$self_PID" | grep -v "xfce4-terminal" | awk '{print $2}')
        echo "Got old terminal process(es):"
        ps -ef | grep "direct.sh.*[$first_char]$rest_chars\$" | grep -v "$self_PID" | grep -v "xfce4-terminal"
	fi
	
	if [[ ! -z "$program_PIDS" || ! -z "$terminal_PIDS" ]]
	then
        OIFS="$IFS"
        while IFS=$' \t\n' read -ra KILLPID <&9 || [ -n "$KILLPID" ] 
        do
	        echo "Kill the program process: $KILLPID!"
		    kill -15 "$KILLPID" >/dev/null 2>&1
        done 9<<< "$program_PIDS"
        
        while IFS=$' \t\n' read -ra KILLPID <&9 || [ -n "$KILLPID" ] 
        do
	        echo "Kill the terminal process: $KILLPID!"
		    kill -15 "$KILLPID" >/dev/null 2>&1
        done 9<<< "$terminal_PIDS"
        IFS="$OIFS"
	else
		echo "No old program is not running!"
	fi
	
    #wait for roscore to kill current roscore and restart
    sleep 3
    
    roscore_terminal_PID=$(ps -ef | grep "direct.*[r]oscore" | grep -v "$self_PID" | awk '{print $2}')
    roscore_PID=$(ps -ef | grep "python.*[r]oscore" | awk '{print $2}')
    rosmaster_PID=$(ps -ef | grep "[r]osmaster --core -p 11311" | awk '{print $2}')
    rosout_PID=$(ps -ef | grep "lib/rosout/[r]osout" | awk '{print $2}')
    
    #if terminal PID detects multiple line, then it is the old one still not killed, and we wait.
	while [[ -z "$roscore_terminal_PID" || -z "$roscore_PID" || -z "$rosmaster_PID" || -z "$rosout_PID" || $roscore_terminal_PID == *$'\n'* ]]
    do
		echo "Wait for ROS master to start up!"
		sleep 1
        roscore_terminal_PID=$(ps -ef | grep "direct.*[r]oscore" | grep -v "$self_PID" | awk '{print $2}')
        roscore_PID=$(ps -ef | grep "python.*[r]oscore" | awk '{print $2}')
        rosmaster_PID=$(ps -ef | grep "[r]osmaster --core -p 11311" | awk '{print $2}')
        rosout_PID=$(ps -ef | grep "lib/rosout/[r]osout" | awk '{print $2}')
	done
    echo "Ros master is running, at these PIDs: $rosmaster_PID, $rosout_PID, $roscore_PID, $roscore_terminal_PID!"

else
    roscore_terminal_PID=$(ps -ef | grep "direct.*[r]oscore" | grep -v "$self_PID" | awk '{print $2}')
    roscore_PID=$(ps -ef | grep "python.*[r]oscore" | awk '{print $2}')
    rosmaster_PID=$(ps -ef | grep "[r]osmaster --core -p 11311" | awk '{print $2}')
    rosout_PID=$(ps -ef | grep "lib/rosout/[r]osout" | awk '{print $2}')
    
    while [[ ! -z "$roscore_terminal_PID" || ! -z "$roscore_PID" || ! -z "$rosmaster_PID" || ! -z "$rosout_PID" ]]
    do
        OIFS="$IFS"
        while IFS=$' \t\n' read -ra KILLPID <&9 || [ -n "$KILLPID" ] 
        do
            echo "Kill the old roscore related process: $KILLPID!"
	        kill -15 "$KILLPID" >/dev/null 2>&1
        done 9<<< "$roscore_PID $roscore_terminal_PID $rosout_PID $rosmaster_PID "
        IFS="$OIFS"
        #keep trying until it really killed all existing
        sleep 1
        roscore_terminal_PID=$(ps -ef | grep "direct.*[r]oscore" | grep -v "$self_PID" | awk '{print $2}')
        roscore_PID=$(ps -ef | grep "python.*[r]oscore" | awk '{print $2}')
        rosmaster_PID=$(ps -ef | grep "[r]osmaster --core -p 11311" | awk '{print $2}')
        rosout_PID=$(ps -ef | grep "lib/rosout/[r]osout" | awk '{print $2}')
    done
    echo "No more roscore is running now!"
fi

if [[ "$skip_flag" = false ]]
then
	echo "Starting $program!"
	sleep "$delay"
    $program $param1 $param2
	echo "ready to start again: $program $param1 $param2"
else
	echo "Nothing to do and this terminal window is going to be closed in 90 seconds!"
	sleep 90
	kill -15 "$self_PID"
fi

$SHELL

