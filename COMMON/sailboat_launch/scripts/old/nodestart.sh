#!/bin/bash
function checkNodeValid {
	_node_name=$1
	roscore_PID=$(ps -ef| grep "python.*[r]oscore")
	if [[ ! -z "$roscore_PID" ]]
	then
		if node_ping=$(rosnode ping -c 1 "$_node_name" 2>>/dev/null | grep "xmlrpc reply from")
		then
		    if [[ $node_ping = *"time="* ]]
		    then
			    return 0
		    else
			    echo "Ping to $_node_name failed, ping result is: $node_ping."
		    fi
        else
            #if rosnode ping fails, grep also fails, so this makes sense!
            printLog "rosnode ping execution failed for $_node_name!"
		fi
	fi
	return 1
}

delay="$1"
folder="$2"

if [[ -z "$4" ]]
then
	launch_file="$3"
	node=""
else
	node="$3"
	launch_file="$4"
fi

if [[ -z "$folder" || -z "$launch_file" ]]
then
    echo "Wrong parameters!"
    exit 1
fi

echo "1: delay: $delay"
echo "2: folder: $folder"
echo "3: node: $node"
echo "4: lauchfile: $launch_file"

self_PID=$$
echo "self_PID=$self_PID"

#kill the old running process if any.
if [[ "$folder" = *"demo/map_loading"* ]]
then
	node_PIDS=$(ps -ef | grep "python.*roslaunch.*demo/[m]ap_loading" | awk '{print $2}')
	echo "Got old node process(es):"
	ps -ef | grep "python.*roslaunch.*demo/[m]ap_loading"
	terminal_PIDS=$(ps -ef | grep "nodestart[.]sh.*demo/[m]ap_loading" | grep -v "$self_PID" | grep -v "xfce4-terminal" |  awk '{print $2}')
    echo "Got old terminal process(es):"
    ps -ef | grep "nodestart[.]sh.*demo/[m]ap_loading" | grep -v "$self_PID" | grep -v "xfce4-terminal"
else
    pure_name="${launch_file%.*}"
    file_extension="${launch_file##*.}"
	node_PIDS=$(ps -ef | grep "python.*roslaunch.*$pure_name[.]$file_extension" | awk '{print $2}')
	echo "Got old node process(es):"
	ps -ef | grep "python.*roslaunch.*$pure_name[.]$file_extension"
	terminal_PIDS=$(ps -ef | grep "nodestart[.]sh.*$pure_name[.]$file_extension" | grep -v "$self_PID" | grep -v "xfce4-terminal" | awk '{print $2}')
    echo "Got old terminal process(es):"
    ps -ef | grep "nodestart[.]sh.*$pure_name[.]$file_extension" | grep -v "$self_PID"
fi

socket_check=false
skip_flag=false

if [[ ! -z "$node_PIDS" || ! -z "$terminal_PIDS" ]]
then
	if [[ "$launch_file" = *"zgwgateway"* ]]
	then
#		checkNodeValid "zgw2ros_total" && skip_flag=true
		if [[ "$skip_flag" = false ]]
		then
			socket_check=true
			socket_port=4000
		fi
#	elif [[ $launch_file = *"fbm_zgw_mkt_controller"* ]]
#	then
#		checkNodeValid "fbm_button" && checkNodeValid "zgw_mode_control" && checkNodeValid "ros_monitor" && skip_flag=true
	fi

	if [[ "$skip_flag" = false ]]
	then
	    if [[ ! -z "$node_PIDS" || ! -z "$terminal_PIDS" ]]
	    then
            OIFS="$IFS"
            while IFS=$' \t\n' read -ra KILLPID <&9 || [ -n "$KILLPID" ] 
            do
	            echo "Kill the node process: $KILLPID!"
		        kill -15 "$KILLPID" >/dev/null 2>&1
            done 9<<< "$node_PIDS"
            
            while IFS=$' \t\n' read -ra KILLPID <&9 || [ -n "$KILLPID" ] 
            do
	            echo "Kill the terminal process: $KILLPID!"
		        kill -15 "$KILLPID" >/dev/null 2>&1
            done 9<<< "$terminal_PIDS"
            IFS="$OIFS"
        else
            echo "No old ROS node is not running!"
        fi

		while [[ "$socket_check" = true ]]
		do
			socket_PID=`lsof -t -i :$socket_port|head -n 1`
			if [[ -z  "$socket_PID" ]]
			then
				socket_check=false
			else
				echo "The required socket for $launch_file has been occupied, and let's kill the old one to start new one!"
				kill -15 $socket_PID
			fi
			sleep 1
		done
	else
		echo "Nodes $launch_file are running good, and we will not restart them!"
	fi
fi

#wait for roscore to kill current roscore and restart
sleep 3

#rosmaster_PID=`lsof -t -i :11311|head -n 1`
roscore_terminal_PID=$(ps -ef | grep "direct.*[r]oscore" | grep -v "$self_PID" | awk '{print $2}')
roscore_PID=$(ps -ef | grep "python.*[r]oscore" | awk '{print $2}')
rosmaster_PID=$(ps -ef | grep "[r]osmaster --core -p 11311" | awk '{print $2}')
rosout_PID=$(ps -ef |grep "lib/rosout/[r]osout" | awk '{print $2}')

#if terminal PID detects multiple line, then it is the old one still not killed, and we wait.
while [[ -z "$roscore_terminal_PID" || -z "$roscore_PID" || -z "$rosmaster_PID" || -z "$rosout_PID" || $roscore_terminal_PID == *$'\n'* ]]
do
	echo "Wait for ROS master to start up!"
	sleep 1
    roscore_terminal_PID=$(ps -ef | grep "direct.*[r]oscore" | awk '{print $2}')
    roscore_PID=$(ps -ef | grep "python.*[r]oscore" | awk '{print $2}')
    rosmaster_PID=$(ps -ef | grep "[r]osmaster --core -p 11311" | awk '{print $2}')
    rosout_PID=$(ps -ef | grep "lib/rosout/[r]osout" | awk '{print $2}')
done
echo "Ros master is running, at these PIDs: $rosmaster_PID, $rosout_PID, $roscore_PID, $roscore_terminal_PID!"

if [[ "$skip_flag" = false ]]
then
	sleep "$delay"
	cd "$folder"

	if [[ -z "$node" ]]
	then
		roslaunch "$launch_file"
		cd "$folder"
		echo "Use this command to restart: roslaunch $folder/$launch_file"

	else
		roslaunch "$node" "$launch_file"
		cd "$folder"
		echo "Use this command to restart: roslaunch $folder/$node $launch_file"
	fi
else
	echo "Nothing to do and this terminal window is going to be closed in 90 seconds!"
	sleep 90
	kill -15 "$self_PID"
fi

$SHELL

