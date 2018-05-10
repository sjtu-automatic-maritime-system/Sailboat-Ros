nohup rosrun rviz rviz -d rviz/sim.rviz &
sleep 3s
nohup rosrun rqt_gui rqt_gui --perspective-file rqt/sim.perspective &
sleep 3s