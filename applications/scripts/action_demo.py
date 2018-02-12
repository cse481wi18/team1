#!/usr/bin/env python

import fetch_api
import rospy

import sys, select, termios, tty
import perception

# from perception import ProgramManager

msg = """
Welcome to the Action Creator!
Commands:
  list: List saved poses for the current action.
  save <id>: Save the robot's current pose as a step in the current action. The <frame> is relative to the base_link or fiducial <id>
  create <name>: Begins the creation process of an action. You can save poses after creating an action to build up the action.
  open: Opens the gripper
  close: closes the gripper
  load <name>: loads the action saved as <name>
  run <name>: runs the action saved as <name>
  end: finish editing the current action and save the action
  delete: delete the current action and all poses in it
  tags: get the ids of all markers currently detected
  help: Show this list of commands
  quit: end this program
"""

commands = {'list', 'save', 'create', 'end', 'delete', 'open', 'close', 'load', 'run', 'help', 'quit', 'tags'}


def getLine():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        line = sys.stdin.readlines()
    else:
        line = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return line

def wait_for_time():   
    """Wait for simulated time to begin.
    """                
    while rospy.Time().now().to_sec() == 0:                                                            
        pass      


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('perception_front_end')

    wait_for_time()

    program_manager = perception.ProgramManager()
    name = None
    
    try:
        print msg
        while (1):
            user_commands = raw_input().split(' ')
            if user_commands[0] not in commands:
                print "Sorry, that is not a command. Type 'help' to see commands"
            elif user_commands[0] == 'quit':
                exit()
            elif user_commands[0] == 'create':
                if name is not None:
                    print "Delete current action before creating a new one"
                elif len(user_commands) != 2:
                    print "Create requires a <name> to name the action"
                else:
                    result = program_manager.create_program(user_commands[1])
                    if result == -1:
                        print "Unable to create new action"
                    else:
                        name = user_commands[1]
                        print "Created new action " + name
            elif user_commands[0] =='open':
                if name is None:
                    print "Create a new program or load an existing one before editing the robot's pose"
                else:
                    result = program_manager.open_gripper()
                    if result == -1:
                        print "Unable to open gripper"
                    else:
                        print "Opened gripper"
            elif user_commands[0] == 'close':
                if name is None:
                    print "Create a new action or load an existing one before editing the robot's pose"
                else:
                    result = program_manager.close_gripper()
                    if result == -1:
                        print "Unable to close gripper"
                    else:
                        print "Closed gripper"
            elif user_commands[0] == 'list':
                if name is None:
                    print "Create a new action or load an existing one before listing poses"
                else:
                    print program_manager.list_poses()
            elif user_commands[0] == 'save':
                if name is None:
                    print "Create a new action or load an existing one before saving new poses"
                elif len(user_commands) != 2:
                    print "Save requires a frame reference, either base_link or fiducial id"
                else:
                    result = program_manager.save_pose(user_commands[1])
                    if result == -1:
                        print "Unable to save current pose in reference frame " + user_commands[1]
                    else:
                        print "Saved current pose in reference frame " + user_commands[1] + " to action"
            elif user_commands[0] == 'load':
                if name is not None:
                    print "Save or delete current poses before loading a new action"
                elif len(user_commands) != 2:
                    print "Load requires a <name> of the action to load"
                else:
                    result = program_manager.load_program(user_commands[1])
                    if result == -1:
                        print "Unable to load action" + user_commands[1]
                    else:
                        print "Loaded action " + user_commands[1]
                        name = user_commands[1]
            elif user_commands[0] == 'delete':
                if name is None:
                    print "No action to delete"
                else:
                    result = program_manager.delete_current_program()
                    if result == -1:
                        print "Unable to delete current action"
                    else:
                        print "Deleted current action"
                        name = None
            elif user_commands[0] == 'end':           
                if name is None:
                    print "No action to end"
                else:
                    result = program_manager.end_program()
                    if result == -1:
                        print "Unable to end current action"
                    else:
                        print "Ended current action"
                        name = None
            elif user_commands[0] == 'run':           
                if len(user_commands) != 2:
                    print "Run requires a <name> of the action to run"
                else:
                    result = program_manager.run_program(user_commands[1])
                    if result == -1:
                        print "Unable to run action named " + user_commands[1]
                    else:
                        print "Completed action named " + user_commands[1]
            elif user_commands[0] == 'tags':           
                result = program_manager.get_tags()
                for tag in result:
                    print tag.id
            elif user_commands[0] == "help":
                print msg
            
    except Exception as e:
        rospy.logerr('{}'.format(e))
    finally:
        if name is not None:
            program_manager.end_program()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
