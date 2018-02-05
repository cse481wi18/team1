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
import numpy


QUEUE_SIZE = 10
X_OFFSET = 0.166 # gripper_link is 0.166 in front of wrist_roll_link in x direction

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
        self._listener = tf.TransformListener() # to compute transforms

    def start(self):
        gripper_im = InteractiveMarker()
        gripper_im.header.frame_id = "base_link"
        gripper_im.name = "Gripper"
        gripper_im.description = "Gripper"
        gripper_im.scale = .35

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
        gripper_marker.pose.position.x = X_OFFSET


        gripper_control = InteractiveMarkerControl()
        gripper_control.name = "gripper control"
        gripper_control.markers.append(copy.deepcopy(gripper_marker))
        gripper_control.always_visible = True
    

        # left finger marker 
        l_finger_marker = Marker()
        l_finger_marker.type = Marker.MESH_RESOURCE
        l_finger_marker.mesh_resource = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
        l_finger_marker.color.r = 1.0
        l_finger_marker.color.g = 0.0
        l_finger_marker.color.b = 0.0
        l_finger_marker.color.a = 1.0
        l_finger_marker.pose.position.x = X_OFFSET
   
        gripper_control.markers.append(copy.deepcopy(l_finger_marker))

         #right finger marker
        r_finger_marker = Marker()
        r_finger_marker.type = Marker.MESH_RESOURCE
        r_finger_marker.mesh_resource = 'package://fetch_description/meshes/r_gripper_finger_link.STL'
        r_finger_marker.color.r = 1.0
        r_finger_marker.color.g = 0.0
        r_finger_marker.color.b = 0.0
        r_finger_marker.color.a = 1.0
        r_finger_marker.pose.position.x = X_OFFSET

        gripper_control.markers.append(copy.deepcopy(r_finger_marker))
        gripper_im.controls.append(copy.deepcopy(gripper_control))
        dof_controls = self._make_6dof_controls()
        gripper_im.controls.extend(copy.deepcopy(dof_controls))
      
        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    # Todo: implement these??
    
    # def make_open_gripper_marker(self, pose):
    
    # def make_gripper_marker(self, pose, finger_distance):
    
    # def make closed_gripper_marker(self, pose):

    # feedback pose is in the base_link frame of ref so we can call move_to_arm
    def handle_feedback(self, feedback):

        if (feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT):
            if feedback.menu_entry_id == 1:
                self._gripper.open()
            elif feedback.menu_entry_id == 2:
                self._gripper.close()
            else: 
                ps = PoseStamped()
                ps.header.frame_id = 'base_link'
                ps.pose = feedback.pose

                error = self._arm.move_to_pose(ps)

                if error is None:
                    self._change_gripper_color(True, ps)
                else:
                    self._change_gripper_color(False, ps)

        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            print feedback.pose
            ps = PoseStamped()
            
            ps.header.frame_id = 'base_link'
            ps.pose = feedback.pose
            
            error = self._arm.compute_ik(ps)
            if error is True:
                self._change_gripper_color(True, ps)
            else:
                rospy.loginfo("Could not find path to move arm")
                self._change_gripper_color(False, ps)
        else:
            pass

    # pose = PoseStamped
    # Marker has a Pose field
    def _change_gripper_color(self, success, pose):
        gripper_im = self._im_server.get("Gripper")

        if success:
            for control in gripper_im.controls:
                for marker in control.markers:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0  
                 
        else: 
            for control in gripper_im.controls:
                for marker in control.markers:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0

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
        control.orientation.y=  1
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

    # pose is a Pose, not PoseStamped
    # pose has position (Point) and orientation (Quaternion)
    def _transform_gripper_position(self, pose):
        # have to do math "backwards" because of matrix multiplication

        # original position in the base link frame 
        b_pose = (pose.position.x, pose.position.y, pose.position.z, 1)

        # original rotation in base link frame
        q = pose.orientation
        print "---------"
        print q
        print "---------"
        b_rotation = tft.quaternion_matrix([q.x, q.y, q.z, q.w]) 

        # BASE_LINK TO GRIPPER_LINK MATRIX
        # transformation between base_link and gripper_link frames
        (position, quaternion) = self._listener.lookupTransform('base_link', 'gripper_link', rospy.Time(0))
        # turn this transformation into a transformation matrix
        b_g_matrix = tft.quaternion_matrix(quaternion)
        # set 4th col of transformation matrix to be translation (aka position) vector
        b_g_matrix[:, 3] = (pose.position.x, pose.position.y, pose.position.z, 1) 

        # GRIPPER_LINK TO BASE_LINK MATRIX
        # inverse to get matrix transformation from gripper_link to base_link
        g_b_matrix = numpy.linalg.inv(b_g_matrix)

        # X_OFFSET MATRIX
        # apply x direction offset in the gripper_link frame to account for wrist_link end effector
        offset_matrix = numpy.identity(4)
        offset_matrix[0,3] = X_OFFSET
     
        temp1 = numpy.dot(b_g_matrix, b_pose) # 4x1
        temp2 = numpy.dot(offset_matrix, temp1) #4x1
        temp3 = numpy.dot(b_rotation, temp2)
        temp4 = numpy.dot(g_b_matrix, temp3) #4x1

        # this is the position vector that the gripper should be in
        return (temp4[0], temp4[1], temp4[2])




class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        gripper_im = InteractiveMarker()
        gripper_im.header.frame_id = "base_link"
        gripper_im.name = "Gripper"
        gripper_im.description = "Gripper"
        gripper_im.scale = .35
        
        # menu entries
        menu1 = MenuEntry()
        menu1.id = 1
        menu1.parent_id = 0
        menu1.title = "Pick up object"
        menu1.command_type = 0
        
        gripper_im.menu_entries.append(menu1)

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
        gripper_marker.pose.position.x = -X_OFFSET
    


        gripper_control = InteractiveMarkerControl()
        gripper_control.name = "gripper control"
        gripper_control.markers.append(copy.deepcopy(gripper_marker))
        gripper_control.always_visible = True
    

        # left finger marker 
        l_finger_marker = Marker()
        l_finger_marker.type = Marker.MESH_RESOURCE
        l_finger_marker.mesh_resource = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
        l_finger_marker.color.r = 1.0
        l_finger_marker.color.g = 0.0
        l_finger_marker.color.b = 0.0
        l_finger_marker.color.a = 1.0
        l_finger_marker.pose.position.x = -X_OFFSET
   


        gripper_control.markers.append(copy.deepcopy(l_finger_marker))

         #right finger marker
        r_finger_marker = Marker()
        r_finger_marker.type = Marker.MESH_RESOURCE
        r_finger_marker.mesh_resource = 'package://fetch_description/meshes/r_gripper_finger_link.STL'
        r_finger_marker.color.r = 1.0
        r_finger_marker.color.g = 0.0
        r_finger_marker.color.b = 0.0
        r_finger_marker.color.a = 1.0
        r_finger_marker.pose.position.x = -X_OFFSET

        gripper_control.markers.append(copy.deepcopy(r_finger_marker))


        #object marker
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.color.r = 0.7
        box_marker.color.g = 0.7
        box_marker.color.b = 0.0 
        box_marker.color.a = 1.0
        box_marker.scale.x = 0.05
        box_marker.scale.y = 0.05
        box_marker.scale.z = 0.05
        # box_marker.pose.position.x = X_OFFSET + 0.01
        box_marker.id = 5

        gripper_control.markers.append(copy.deepcopy(box_marker))
   
        
        #doesn't work to translate the  marker positions

        gripper_im.controls.append(copy.deepcopy(gripper_control))
        dof_controls = self._make_6dof_controls()
        gripper_im.controls.extend(copy.deepcopy(dof_controls))
      
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
        control.orientation.y=  1
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

        # pose = PoseStamped
    # Marker has a Pose field
    def _change_gripper_color(self, success, pose):
        gripper_im = self._im_server.get("Gripper")

        if success:
            for control in gripper_im.controls:
                for marker in control.markers:
                    if marker.id != 5: # cube
                        marker.color.r = 0.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                        marker.color.a = 1.0  

        else: 
            for control in gripper_im.controls:
                for marker in control.markers:
                    if marker.id != 5: # cube
                        marker.color.r = 1.0
                        marker.color.g = 0.0
                        marker.color.b = 0.0
                        marker.color.a = 1.0

        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()
        

    def handle_feedback(self, feedback):

        if (feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT):
            if feedback.menu_entry_id == 1:
                ps = PoseStamped()

                # TODO: The pose is given to us in gripper_link frame 
                # but we need it to be in base_link frame so transformation??
                ps.header.frame_id = 'base_link'

                #adjusts to be directly on top of the (incorrectly placed) marker
                ps.pose = self._object_pose_to_gripper_pose(feedback.pose)

                error = self._arm.move_to_pose(ps)

                if error is None:
                    print ps
                    self._change_gripper_color(True, ps)
                    print "closing gripper"
                    self._gripper.close()
                    print "raising arm"

                    raised_ps = PoseStamped()
                    raised_ps.header.frame_id = 'base_link'

                    #adjusts to be directly on top of the (incorrectly placed) marker
                    raised_ps.pose = feedback.pose
                    raised_ps.pose.position.z = feedback.pose.position.z + .05

                    print "moving to pose"
                    print raised_ps
                    error = self._arm.move_to_pose(raised_ps)
                    if error is not None:
                        print error
                else:
                    print "could not move"
                    self._change_gripper_color(False, ps)
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            print feedback.pose
            ps = PoseStamped()

            # TODO: The pose is given to us in gripper_link frame 
            # but we need it to be in base_link frame so transformation??
            
            ps.header.frame_id = 'base_link'
            ps.pose = self._object_pose_to_gripper_pose(feedback.pose)
            
            
            error = self._arm.compute_ik(ps)
           
            if error is True:
                self._change_gripper_color(True, ps)
            else:
                rospy.loginfo("Could not find path to move arm")
                self._change_gripper_color(False, ps)
        else:
            pass


    # pose is a Pose, not PoseStamped
    # pose has position (Point) and orientation (Quaternion)
    # given pose of object, return pose of gripper in base link frame
    def _object_pose_to_gripper_pose(self, pose):
        # have to do math "backwards" because of matrix multiplication

        # transformation of object in base link frame  
        q = pose.orientation
        b_T_o = tft.quaternion_matrix([q.x, q.y, q.z, q.w]) 
        b_T_o[:,3] = (pose.position.x, pose.position.y, pose.position.z, 1) 

        # transformation of gripper in object frame
        o_T_g = numpy.identity(4)
        o_T_g[0, 3] = -2 * X_OFFSET

        # want transformation of gripper in base link frame, aka b_T_g
        b_T_g = numpy.dot(b_T_o, o_T_g)

        # transform this back into a pose 
        result = Pose()
        result.position = Point(b_T_g[0, 3], b_T_g[1, 3], b_T_g[2, 3])
        temp = tft.quaternion_from_matrix(b_T_g)
        result.orientation = Quaternion(temp[0], temp[1], temp[2], temp[3])
        
        return result

       



def main():
    rospy.init_node('gripper_teleop_demo')
    wait_for_time()

    gripper = fetch_api.Gripper()
    arm = fetch_api.Arm()

  #  im_server = InteractiveMarkerServer('gripper_im_server')
    # for running on real robot
    im_server = InteractiveMarkerServer('gripper_im_server', q_size = 2)
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server')
    # auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server', q_size = 2)
    teleop = GripperTeleop(arm, gripper, im_server)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)

    teleop.start()
    auto_pick.start()
    rospy.spin()

if __name__ == "__main__":
    main()