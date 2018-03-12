#!/bin/bash

xterm -title "Rviz" -hold -e 'sleep 5; setrobot astro; rosrun rviz rviz -d /home/cse481cwi18/Desktop/team1.rviz' &

xterm -title "MoveIt" -hold -e 'ssh team1@astro; roslaunch fetch_api move_group.launch' &

xterm -title "Nav Map" -hold -e 'ssh team1@astro; roslaunch applications sieg_nav.launch' &

xterm -title "Perception" -hold -e 'roslaunch perception table_recognizer.launch is_sim:=false' &

xterm -title "Action Demo" -hold -e 'sleep 10; rosrun applications action_demo.py' &

xterm -title "Nav Demo" -hold -e 'sleep 10; ssh team1@astro; rosrun applications navigation_demo.py' &
