#!/usr/bin/env python

import fetch_api
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, MenuEntry
from visualization_msgs.msg import Marker
import copy
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, PoseStamped
from std_msgs.msg import Header, ColorRGBA
from joint_state_reader import JointStateReader
import tf
import tf.transformations as tft


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
        self.current_pose = None

    def start(self):

        gripper_im = InteractiveMarker()
        gripper_im.header.frame_id = "gripper_link"
        gripper_im.name = "Gripper"
        gripper_im.description = "Gripper"
        gripper_im.scale = .5
        gripper_im.pose.position.x = .166
        
        # menu entries
        menu1 = MenuEntry()
        menu1.id = 1
        menu1.parent_id = 0
        menu1.title = "Open Gripper"
        menu1.command_type = 0

        menu2 = MenuEntry()
        menu2.id = 2
        menu2.parent_id = 0
        menu2.title = "Close Gripper"
        menu2.command_type = 0

        menu3 = MenuEntry()
        menu3.id = 3
        menu3.parent_id = 0
        menu3.title = "Go To Gripper Pose"
        menu3.command_type = 0
        
        gripper_im.menu_entries.append(menu1)
        gripper_im.menu_entries.append(menu2)
        gripper_im.menu_entries.append(menu3)

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
        dof_controls = self._make_6dof_controls()
        gripper_im.controls.extend(dof_controls)
      
        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    # Todo: implement these??
    
    # def make_open_gripper_marker(self, pose):
    
    # def make_gripper_marker(self, pose, finger_distance):
    
    # def make closed_gripper_marker(self, pose):

    def handle_feedback(self, feedback):
        print "call back"
        
        if (feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT):
            if feedback.menu_entry_id == 1:
                self._gripper.open()
            elif feedback.menu_entry_id == 2:
                self._gripper.close()
            else:
                print feedback.pose
                ps = PoseStamped()

                # TODO: The pose is given to us in gripper_link frame 
                # but we need it to be in base_link frame so transformation??
                ps.header.frame_id = 'base_link'

                ps.pose = feedback.pose

                error = self._arm.move_to_pose(ps)
                if error is None:
                    self._change_gripper_color(True)
                else:
                    self._change_gripper_color(False)

        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            print feedback.pose
            ps = PoseStamped()

            # TODO: The pose is given to us in gripper_link frame 
            # but we need it to be in base_link frame so transformation??

            ps.header.frame_id = 'base_link'
            ps.pose = feedback.pose
            error = self._arm.check_pose(ps, allowed_planning_time=10.0)
            if error is None:
                self._change_gripper_color(True, ps)
            else:
                rospy.loginfo("Could not find path to move arm")
                self._change_gripper_color(False, ps)
        else:
            print "IDK"


    def _change_gripper_color(self, success, pose):
        gripper_im = self._im_server.get("Gripper")
        if success:
            for control in gripper_im.controls:
                for marker in control.markers:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0
                    marker.pose = pose.pose

                control.markers.append(copy.deepcopy(marker))
            gripper_im.controls.append(copy.deepcopy(control))
        else: 
            for control in gripper_im.controls:
                for marker in control.markers:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0
                    marker.pose = pose.pose
        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()
    

    # returns list of InteractiveMarkerControl
    def _make_6dof_controls(self): 
        controls_list = []

        control = InteractiveMarkerControl()
        control.always_visible = True

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

        return controls_list

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

    gripper = fetch_api.Gripper()
    arm = fetch_api.Arm()

    im_server = InteractiveMarkerServer('gripper_im_server')
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server')
    teleop = GripperTeleop(arm, gripper, im_server)
  # auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)

    teleop.start()
   # auto_pick.start()
    rospy.spin()

if __name__ == "__main__":
    main()