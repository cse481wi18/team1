from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

class NavPath(object):
    def __init__(self):
        self._path = []
        self.pub = rospy.Publisher('visualization_marker', Marker)

    def callback(self, msg):
        rospy.loginfo(msg)
        if SOME_CONDITION:
            self._path.append(msg.pose.pose.position)
    
    def show_path_in_rviz(marker_publisher, text):
        marker = Marker(
                type=Marker.SPHERE_LIST,
                id=0,
                pos = 
                points = path,
                scale=Vector3(0.06, 0.06, 0.06),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
    marker_publisher.publish(marker)

def main():
    # ...setup stuff...
    nav_path = NavPath()
    rospy.Subscriber('odom', Odometry, nav_path.callback)
    rospy.spin()
