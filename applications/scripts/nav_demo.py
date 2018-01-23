#!/usr/bin/env python
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry

import math
import rospy

class NavPath(object):
    DISTANCE = .5

    def __init__(self):
        self._path = []
        self.pub = rospy.Publisher('visualization_marker', Marker, queue_size=5)

    def callback(self, msg):
        current_point = msg.pose.pose.position
        
        first = False

        if len(self._path) > 0:
            previous_point = self._path[-1]
        else:
            first = True

        if first or self.distance_between_points(current_point.x, current_point.y, previous_point.x, previous_point.y) > self.DISTANCE:
                self._path.append(msg.pose.pose.position)
                self.show_path_in_rviz()

    def show_path_in_rviz(self):
        marker = Marker(
                type=Marker.SPHERE_LIST,
                id=0,
                points = self._path,
                pose=Pose(Point(0,0,0), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.06, 0.06, 0.06),
                header=Header(frame_id='odom'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
        self.pub.publish(marker)

    def distance_between_points(self, x1, y1, x2, y2):
        sq1 = (x1-x2)*(x1-x2)
        sq2 = (y1-y2)*(y1-y2)
        return math.sqrt(sq1+sq2)

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def main():
    rospy.init_node('nav_node')
    wait_for_time()
    nav_path = NavPath()
    rospy.Subscriber('odom', Odometry, nav_path.callback)
    rospy.spin()

if __name__ == '__main__':
  main()
