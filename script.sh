#!/bin/bash
xterm -title "Roscore" -hold -e 'roscore' &
xterm -title "Gazebo" -hold -e 'sleep 1; roslaunch fetch_gazebo playground.launch' &
xterm -title "Publish cloud" -hold -e 'sleep 7; cd /home/team1/catkin_ws/src/cse481wi18; rosrun applications publish_saved_cloud.py table_hallway_back.bag' &
xterm -title "Rviz" -hold -e 'sleep 5; rosrun rviz rviz' &
xterm -title "MoveIt" -hold -e 'sleep 7; roslaunch fetch_api move_group.launch' &
xterm -title "Perception" -hold -e 'sleep 10; roslaunch perception table_recognizer.launch is_sim:=true' &
xterm -title "Action Demo" -hold -e 'sleep 10; rosrun applications action_demo.py' &
