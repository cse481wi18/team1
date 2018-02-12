#!/usr/bin/env python                                                                              
                                                                                                       
import rospy
import pickle    
import fetch_api
import tf
import actionlib
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
import tf.transformations as tft
import numpy as np
from robot_controllers_msgs.msg import ControllerState, QueryControllerStatesGoal, QueryControllerStatesAction


class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers

class ProgramManager(object): 
    def __init__(self):
        self._current_program_name = None
        self._current_program = []
        self._in_progress = False # whether a program is currently being saved

        self._arm = fetch_api.Arm()
        self._gripper = fetch_api.Gripper()
        self._reader = ArTagReader()
        self._subscriber = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, callback=self._reader.callback) # Subscribe to AR tag poses, use reader.callback

        self._controller_client = actionlib.SimpleActionClient('query_controller/controller_state', QueryControllerStatesAction)

    # Starts a program. Must be called before poses are saved.
    # name is the name of the program
    def create_program(self, name):
        if self._in_progress:
            return -1 # program saving not in progress
        
        self._in_progress = True 
        self._current_program_name = name
        self._listener = tf.TransformListener()
        self._relax_arm_controller()
    

    # loads program into memory to continue editing it
    def load_program(self, name): 
        self._in_progress = True 
        self._current_program_name = name
        program = self._pickle_load(name)
        if program == -1:
            return -1
        self._current_program = program
        self._relax_arm_controller()
    

    # removes all poses from current program but does not end it
    def delete_current_program(self):
        self._current_program = []
        self._in_progress = False 
        self._current_program_name = None
        return 0

    # appends current pose, relative to frame, to the current program
    # returns 0 on success, -1 on error
    def save_pose(self, frame):
        if not self._in_progress:
            return -1 # program saving not in progress
        pose = Pose()
        relative = ""
        if frame == 'base_link': 
            (position, quaternion) = self._listener.lookupTransform('base_link', 'wrist_roll_link', rospy.Time(0))
            pose.position = position
            pose.orientation = quaternion
            relative = 'base_link' 
        else : # some tag
            current_marker = None
            for marker in self._reader.markers:
                print marker.id
                if int(frame) == marker.id:
                    current_marker = marker
            if not current_marker:
                print "Could not find marker " + frame
                return -1
            


            # Get transfrom matrix from base to wrist 
            (pos_b, quat_b) = self._listener.lookupTransform('base_link', 'wrist_roll_link', rospy.Time(0))
    
            b_w_matrix = tft.quaternion_matrix(quat_b)
            # set 4th col of transformation matrix to be translation (aka position) vector
            pos_b.append(1)
            b_w_matrix[:, 3] = pos_b 

            # current_marker has a pose relative to base frame, aka b_T_tag. We need tag_T_b.
            #Get transfromation matrix from tag to base and invert it
            t_pos = current_marker.pose.pose.position
 
            # original marker rotation in base link frame
            t_q = current_marker.pose.pose.orientation
        
            # b_t_matrix is b_T_tag
            b_t_matrix = tft.quaternion_matrix([t_q.x, t_q.y, t_q.z, t_q.w]) 
            b_t_matrix[:, 3] = (t_pos.x, t_pos.y, t_pos.z, 1) 

            # t_b_matrix is tag_T_b
            t_b_matrix= np.linalg.inv(b_t_matrix)

            # multiply t_b by b_w to get tag_T_w
            t_w_matrix = np.dot(t_b_matrix, b_w_matrix)
        
            pose.position = Point(t_w_matrix [0, 3], t_w_matrix [1, 3], t_w_matrix [2, 3])
            temp = tft.quaternion_from_matrix(t_w_matrix)
            pose.orientation = Quaternion(temp[0], temp[1], temp[2], temp[3])
            relative = frame
          

        self._current_program.append((pose, relative))

        return 0
    
    # special case of save_pose
    def open_gripper(self):
        if not self._in_progress:
            return -1 # program saving not in progress
        
        self._current_program.append(('open_gripper', 'base_link'))

        return 0
    
    # special case of save_pose
    def close_gripper(self):
        if not self._in_progress:
            return -1 # program saving not in progress
        
        self._current_program.append(('close_gripper', 'base_link'))
        return 0

    
    # lists the poses in the current program
    def list_poses(self):
        return self._current_program


    def end_program(self):
        self._pickle_dump(self._current_program_name, self._current_program)
        self._current_program_name = None
        self._current_program = []
        self._in_progress = False

    # TODO
    # executes program
    # returns -1 on error
    def run_program(self, name): 
        program = self._pickle_load(name)
        if program == -1:
            return -1
        self._start_arm_controller()


        for (pose, relative) in program:
            if pose == 'open_gripper': 
                self._gripper.open()
            elif pose == 'close_gripper':
                self._gripper.close()
            else: 
                ps = PoseStamped()
                ps.header.frame_id = 'base_link'
                # need to create new pose and explicitly assign fields since pickle didn't store x/y/z field names
     
                
        
                if relative == 'base_link':
                    temp_pose = Pose()
                    temp_pose.position.x = pose.position[0]
                    temp_pose.position.y = pose.position[1]
                    temp_pose.position.z = pose.position[2]
                    temp_pose.orientation.x = pose.orientation[0]
                    temp_pose.orientation.y = pose.orientation[1]
                    temp_pose.orientation.z = pose.orientation[2]
                    temp_pose.orientation.w = pose.orientation[3]
                    ps.pose = temp_pose
                    print ps

                    error = self._arm.move_to_pose(ps)
                    if error is not None:
                        rospy.logerr(error)
                else: # relative to some tag
                    tag_w_pose = pose # temp_pose is tag_T_w, wrist relative to tag
                    

                    current_marker = None
                    for marker in self._reader.markers:
                        print marker.id
                        if int(relative) == marker.id:
                            current_marker = marker
                    if current_marker is None:
                        print "Could not find marker " + relative
                        return -1


                    # Get transfrom matrix from base to wrist 
                    (tag_w_pos, tag_w_quat) = tag_w_pose.position, tag_w_pose.orientation
                    t_w_matrix = tft.quaternion_matrix([tag_w_quat.x, tag_w_quat.y, tag_w_quat.z, tag_w_quat.w])
                   
                    # set 4th col of transformation matrix to be translation (aka position) vector

                    t_w_matrix[:, 3] = (tag_w_pos.x, tag_w_pos.y, tag_w_pos.z, 1) 

                    # t_w_matrix is tag_T_w

                    # find b_T_tag

                    #Get transfromation matrix from base to tag
                    t_pos = current_marker.pose.pose.position


                    # original rotation in base link frame

                    t_q = current_marker.pose.pose.orientation
                                    


                    b_t_matrix = tft.quaternion_matrix([t_q.x, t_q.y, t_q.z, t_q.w]) 
                    b_t_matrix[:, 3] = (t_pos.x, t_pos.y, t_pos.z, 1) 

                    # b_t_matrix is b_T_tag

                    # multiply t_w by b_t       
                    b_w_matrix = np.dot(b_t_matrix, t_w_matrix)

                    temp_pose = Pose()
                    temp_pose.position = Point(b_w_matrix [0, 3], b_w_matrix [1, 3], b_w_matrix [2, 3])
                   
                    # COMMENT THIS BACK IN TO NOT DO THE IDENTITY ORIENTATION
                    # temp = tft.quaternion_from_matrix(b_w_matrix)
                    # temp_pose.orientation = Quaternion(temp[0], temp[1], temp[2], temp[3])

                     # move to pose with identity orientation and position of marker, since marker in arbitrary orientation (says Justin)
                    temp_pose.orientation = Quaternion(0, 0, 0, 1)
                    
                    ps.pose = temp_pose

                
                    error = self._arm.move_to_pose(ps)
                    if error is not None:
                        rospy.logerr(error)

            rospy.sleep(1)

        return 0

    # must call before setting poses/creating program
    def _relax_arm_controller(self):
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = ControllerState.STOPPED
        goal.updates.append(state)
        self._controller_client.send_goal(goal)
        self._controller_client.wait_for_result(rospy.Duration(1)) 
    
    # must call before executing program
    def _start_arm_controller(self):
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = ControllerState.RUNNING
        goal.updates.append(state)
        self._controller_client.send_goal(goal)
        self._controller_client.wait_for_result(rospy.Duration(1))

    # saves current program to a pickle file name
    def _pickle_dump(self, filename, obj):
        pickle.dump(obj, open(filename + ".p", 'wb'))

    # loads a program called filename into current memory 
    def _pickle_load(self, filename):
        try:
            return pickle.load(open(filename + ".p", 'rb'))
        except Exception as e:
            return -1

    
    
