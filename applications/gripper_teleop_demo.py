#!/usr/bin/env python

import fetch_api
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
import copy
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA


QUEUE_SIZE = 10

def wait_for_time():   
    """Wait for simulated time to begin.
    """                
    while rospy.Time().now().to_sec() == 0:                                                            
        pass   

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        print "Adding marker"
        gripper_im = InteractiveMarker()
        gripper_im.header.frame_id = "wrist_roll_link"
        gripper_im.name = "Gripper"

        # to do find this pose
        gripper_im.pose.position.x = 0.166
        gripper_im.pose.position.y = 0
        gripper_im.pose.position.z = 0
        gripper_im.description = "Gripper"

        #gripper marker
        gripper_marker = Marker()
        gripper_marker.type = Marker.MESH_RESOURCE
        gripper_marker.mesh_resource =  'package://fetch_description/meshes/gripper_link.dae'
        gripper_marker.color.r = 1.0
        gripper_marker.color.g = 0.0
        gripper_marker.color.b = 0.0
        gripper_marker.color.a = 1.0

        gripper_control = InteractiveMarkerControl()
        gripper_control.name = "gripper control"
        gripper_control.markers.append(gripper_marker)
        gripper_control.always_visible = True

        gripper_im.controls.append(copy.deepcopy(gripper_control))
        
        # left finger marker 
        l_finger_marker = Marker()
        l_finger_marker.type = Marker.MESH_RESOURCE
        l_finger_marker.mesh_resource = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
        l_finger_marker.color.r = 0.0
        l_finger_marker.color.g = 1.0
        l_finger_marker.color.b = 0.0
        l_finger_marker.color.a = 1.0


        l_finger_control = InteractiveMarkerControl()
        l_finger_control.name = "l_finger_marker control"
        l_finger_control.markers.append(copy.deepcopy(l_finger_marker))
        l_finger_control.always_visible = True

        gripper_im.controls.append(copy.deepcopy(l_finger_control))
       
        #right finger marker
        r_finger_marker = Marker()
        r_finger_marker.type = Marker.MESH_RESOURCE
        r_finger_marker.mesh_resource = 'package://fetch_description/meshes/r_gripper_finger_link.STL'
        r_finger_marker.color.r = 0.0
        r_finger_marker.color.g = 0.0
        r_finger_marker.color.b = 1.0
        r_finger_marker.color.a = 1.0

        r_finger_control = InteractiveMarkerControl()
        r_finger_control.name = "r_finger_marker control"
        r_finger_control.markers.append(copy.deepcopy(r_finger_marker))
        r_finger_control.always_visible = True

        gripper_im.controls.append(copy.deepcopy(r_finger_control))
       

        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def handle_feedback(self, feedback):
        print "Handleing feedback"


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        # obj_im = InteractiveMarker() ...
        self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)

    def handle_feedback(self, feedback):
        pass


def main():
    rospy.init_node('gripper_teleop_demo')
    wait_for_time()

    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()

    im_server = InteractiveMarkerServer('gripper_im_server')
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server')
    teleop = GripperTeleop(arm, gripper, im_server)
  # auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    print "Adding server"
    teleop.start()
   # auto_pick.start()
    rospy.spin()

if __name__ == "__main__":
    main()