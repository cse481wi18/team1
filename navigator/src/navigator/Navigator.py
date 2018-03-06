#!/usr/bin/env python                                                                                  
                                                                                                       
import rospy 
import pickle
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, PoseStamped, Pose, Point, Quaternion    

class Navigator(object):     

    def _current_pose_callback(self, data):
        # data is of type PoseWithCovarianceStamped, which has a PoseWithCovariance, which has a Pose
        self._last_pose = data.pose.pose # type Pose, which has Point and Quaternion
    

    def __init__(self, timeout):

        # map from table number to table pose
        self._saved_poses = self._pickle_load("poses.p")

        self._last_pose = None
        # acml_pose topic gives current pose
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._current_pose_callback)

        # publish poses to move_base_simple/goal which will move the base (node nav_rviz does this)
        self._pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size = 10)

        self._timeout = timeout

    # name = table number
    # returns -1 if name is not a saved pose, 1 if successfully moved to pose, 0 if couldn't move to pose
    def goto(self, name):
        if name not in self._saved_poses:
            return -1

        msg = PoseStamped()
        msg.pose = self._saved_poses[name]
        msg.header.frame_id = "map"
        self._pub.publish(msg)

        start_time = rospy.get_rostime()

        while self._distance(self._last_pose, msg.pose) > 0.1 and (rospy.get_rostime() - start_time < self._timeout):
            rospy.sleep(0.5) # sleep for 0.5 seconds
        
        if self._distance(self._last_pose, msg.pose) < 0.1:
            return 1
        else:
            return 0

    # pose1 and pose2 are type Pose
    def distance(self, pose1, pose2): 
        x1 = pose1.position.x
        y1 = pose1.position.y 
        x2 = pose2.position.x
        y2 = pose2.position.y
        sq1 = (x1-x2)*(x1-x2)
        sq2 = (y1-y2)*(y1-y2)
        return math.sqrt(sq1+sq2)
    

    def _pickle_dump(self, file):
        pickle.dump(self._saved_poses, open(file, 'wb'))


    def _pickle_load(self, file):
        try:
            self._saved_poses = pickle.load(open(file, 'rb'))
        except Exception as e:
            self._saved_poses = {}



        
        

        
   