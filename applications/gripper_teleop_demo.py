#!/usr/bin/env python

import fetch_api
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
import copy
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from joint_state_reader import JointStateReader
import tf


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

        listener = tf.TransformListener()

        gripper_im = InteractiveMarker()
        gripper_im.header.frame_id = "gripper_link"
        gripper_im.name = "Gripper"
        gripper_im.description = "Gripper"

        #gripper marker
        gripper_marker = Marker()
        gripper_marker.type = Marker.MESH_RESOURCE
        gripper_marker.mesh_resource =  'package://fetch_description/meshes/gripper_link.dae'
        gripper_marker.color.r = 1.0
        gripper_marker.color.g = 0.0
        gripper_marker.color.b = 0.0
        gripper_marker.color.a = 1.0
        gripper_marker.scale.y = 1.05
        gripper_marker.scale.z = 1.05

        gripper_marker.header.frame_id = "gripper_link"

        gripper_control = InteractiveMarkerControl()
        gripper_control.name = "gripper control"
        gripper_control.markers.append(copy.deepcopy(gripper_marker))
        gripper_control.always_visible = True


        # left finger marker 
        l_finger_marker = Marker()
        l_finger_marker.type = Marker.MESH_RESOURCE
        l_finger_marker.mesh_resource = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
        l_finger_marker.color.r = 0.0
        l_finger_marker.color.g = 1.0
        l_finger_marker.color.b = 0.0
        l_finger_marker.color.a = 1.0
        l_finger_marker.header.frame_id = "l_gripper_finger_link"

        gripper_control.markers.append(copy.deepcopy(l_finger_marker))

         #right finger marker
        r_finger_marker = Marker()
        r_finger_marker.type = Marker.MESH_RESOURCE
        r_finger_marker.mesh_resource = 'package://fetch_description/meshes/r_gripper_finger_link.STL'
        r_finger_marker.color.r = 0.0
        r_finger_marker.color.g = 0.0
        r_finger_marker.color.b = 1.0
        r_finger_marker.color.a = 1.0
        r_finger_marker.header.frame_id = "r_gripper_finger_link"

        gripper_control.markers.append(copy.deepcopy(r_finger_marker))

        gripper_im.controls.append(copy.deepcopy(gripper_control))
        gripper_im.controls.append(self._make_6dof_controls())
      
        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
    
        self._im_server.applyChanges()


    def handle_feedback(self, feedback):
        print "call back"
        # if (input.event_type == InteractiveMarkerFeedback.):
        #     rospy.loginfo(input.marker_name + ' was clicked.')
        #     base = fetch_api.Base()
        #     base.go_forward(STEP, .8)


    # returns list of InteractiveMarkerControl
    def _make_6dof_controls(self): 
        controls_list = []

        control = InteractiveMarkerControl()
        
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        controls_list.append(copy.deepcopy(control))

        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        controls_list.append(copy.deepcopy(control))

        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        controls_list.append(copy.deepcopy(control))

        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        controls_list.append(copy.deepcopy(control))

        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        controls_list.append(copy.deepcopy(control))

        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        controls_list.append(copy.deepcopy(control))
 



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