#!/usr/bin/env python                                                                              
                                                                                                       
import rospy    
import pickle

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, PoseStamped, Pose, Point, Quaternion                                                                                  
                                                                                                       
class MapAnnotator(object):                                                                        
    """Listens to /joint_sta        pickle.dump(self._saved_poses, open("poses.p", 'wb'))
tes and provides the latest joint angles.                                  
                                                                                                       
    Usage:                                                                                             
        joint_reader = JointStateReader()                                                              
        rospy.sleep(0.1)                                                                               
        joint_reader.get_joint('shoulder_pan_joint')                                                   
        joint_reader.get_joints(['shoulder_pan_joint', 'shoulder_lift_joint'])                         
    """                                                                                                

    def _callback(self, data):
        # data is of type PoseWithCovarianceStamped, which has a PoseWithCovariance, which has a Pose
        self._last_pose = data.pose.pose # type Pose, which has Point and Quaternion
    
    def __init__(self):
        self._saved_poses = {}
        self._last_pose = None
        # acml_pose topic gives current pose
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._callback)

        # publish poses to move_base_simple/goal which will move the base (node nav_rviz does this)
        self._pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size = 10)

    def save(self, name):
        # wait for first pose message to come in
        while self._last_pose == None: 
            rospy.sleep(.5)

        # map name to current pose, overwriting name if necessary
        self._saved_poses[name] = self._last_pose 

    # returns -1 if name is not a saved pose
    def goto(self, name):
        if name not in self._saved_poses:
            return -1

        msg = PoseStamped()
        msg.pose = self._saved_poses[name]
        msg.header.frame_id = "map"
        self._pub.publish(msg)
        return 0

    # returns list in Python 2, or view object in Python 3
    def list_poses(self):
        return self._saved_poses.keys()

    # returns -1 if name is not a pose
    def delete(self, name): 
        if name not in self._saved_poses:
            return -1

        self._saved_poses.pop(name, None)
        return 0

    def pickle_dump(self):
        print "dumping"
        pickle_out = open("poses.p", 'wb')
        pickle.dumps(self._saved_poses, pickle_out)
        pickle_out.close()
        print "closing"


    def pickle_load(self):
        open("poses.p", 'w+')
        try:
            self._saved_poses = pickle.loads(open("poses.p", 'rb'))
        except EOFError:
            self._saved_poses = {}
        
        

        
   