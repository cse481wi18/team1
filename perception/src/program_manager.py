#!/usr/bin/env python                                                                              
                                                                                                       
import rospy
import pickle    
import fetch_api
import tf

class ProgramManager(object): 
    def __init__(self):
        self._current_program_name = None
        self._current_program = []
        self._in_progress = False # whether a program is currently being saved

        self._arm = fetch_api.Arm()
        self._reader = ArTagReader()
        self._subscriber = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, callback=reader.callback) # Subscribe to AR tag poses, use reader.callback

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
        if self._pickle_load(name) == -1:
            return -1
        self._relax_arm_controller()
    

    # removes all poses from current program but does not end it
    def delete_all_poses(self):
        self._current_program = []
        return 0

    # appends current pose, relative to frame, to the current program
    # returns 0 on success, -1 on error
    def save_pose(self, frame):
        if not self._in_progress:
            return -1 # program saving not in progress

        pose = Pose()
        relative = ""
        if frame == 'base': 
            (position, quaternion) = listener.lookupTransform('gripper_link', 'base_link', rospy.Time(0))
            pose.position = position
            pose.orientation = quaternion 
            relative = 'base' 
        else : # some tag
            pass
        self._current_program.append((pose, relative))

        return 0
    
    # special case of save_pose
    def open_gripper(self):
        if not self._in_progress:
            return -1 # program saving not in progress
        
        self._current_program.append(OPEN POSE HERE)
        return 0
    
    # special case of save_pose
    def close_gripper(self):
        if not self._in_progress:
            return -1 # program saving not in progress
        
        self._current_program.append(CLOSE POSE HERE)
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
        if self._pickle_load(filename) == -1:
            return -1
        self._start_arm_controller()

        for (pose, relative) in self._current_program:
            if relative == "base":
                error = self._arm.move_to_pose(pose)
                if error is not None:
                    rospy.logerr(error)
            else:
                pass
            # DO STUFF HERE TO TRANSFORM POSE TO BASE FRAME
            # error = self._arm.move_to_pose(pose)
            # if error is not None:
            #     rospy.logerr(error)
            rospy.sleep(1)
        
        return 0


    def relax_arm_controller(self):
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = ControllerState.STOPPED
        goal.updates.append(state)
        self._controller_client.send_goal(goal)
        self._controller_client.wait_for_result() 
    

    def start_arm_controller(self):
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = ControllerState.RUNNING
        goal.updates.append(state)
        self._controller_client.send_goal(goal)
        self._controller_client.wait_for_result()

    # saves current program to a pickle file name
    def _pickle_dump(self, filename, obj):
        pickle.dump(obj, open(filename + ".p", 'wb'))

    # loads a program called filename into current memory 
    def _pickle_load(self, filename):
        try:
            self._current_program = pickle.load(open(filename + ".p", 'rb'))
        except Exception as e:
            return -1

    