. /opt/ros/indigo/setup.bash
. ~/catkin_ws/devel/setup.bash

rm -rf labels
mkdir labels
rosrun perception extract_features blue_cup.bag blue_cup
rosrun perception extract_features expo.bag expo
rosrun perception extract_features galaxy_nexus.bag galaxy_nexus
rosrun perception extract_features orange_dog_toy.bag orange_dog_toy
rosrun perception extract_features red_cup.bag red_cup
rosrun perception extract_features turtle.bag turtle
rosrun perception extract_features yellow_dog_toy.bag yellow_dog_toy
mv *_label.bag labels
