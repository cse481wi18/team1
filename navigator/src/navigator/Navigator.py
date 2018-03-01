#!/usr/bin/env python                                                                                  
                                                                                                       
import rospy 
import pickle
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, PoseStamped, Pose, Point, Quaternion    

class Navigator(object):     

     def _callback(self, data):
        # data is of type PoseWithCovarianceStamped, which has a PoseWithCovariance, which has a Pose
        self._last_pose = data.pose.pose # type Pose, which has Point and Quaternion
    
    def __init__(self):

        # map from table number to table pose
        self._saved_poses = self._pickle_load("tables.p")

        self._last_pose = None
        # acml_pose topic gives current pose
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._callback)

        # publish poses to move_base_simple/goal which will move the base (node nav_rviz does this)
        self._pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size = 10)

    # name = table number
    # returns -1 if name is not a saved pose
    def goto(self, name):
        if name not in self._saved_poses:
            return -1

        msg = PoseStamped()
        msg.pose = self._saved_poses[name]
        msg.header.frame_id = "map"
        self._pub.publish(msg)
        return 0
    
    def pickle_dump(self, file):
        pickle.dump(self._saved_poses, open(file, 'wb'))

    def pickle_load(self, file):
        try:
            self._saved_poses = pickle.load(open(file, 'rb'))
        except Exception as e:
            self._saved_poses = {}


        
        

        
   