#!/usr/bin/env python

import fetch_api
import rospy

import sys, select, termios, tty

from map_annotator import MapAnnotator

msg = """
Welcome to the map annotator!
Commands:
  list: List saved poses.
  save <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists.
  delete <name>: Delete the pose given by <name>.
  goto <name>: Sends the robot to the pose given by <name>.
  help: Show this list of commands
"""

commands = {'list', 'save', 'delete', 'goto', 'help', 'quit'}


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

    rospy.init_node('fetch_navigation_key')

    wait_for_time()

    map_annotator = MapAnnotator("/home/team1/map_poses/sim_poses.p")
    map_annotator.pickle_load()

    try:
        print msg
        while (1):
            user_commands = raw_input().split(' ')
            if user_commands[0] not in commands:
                print "You dumbo"
            elif user_commands[0] == 'quit':
                exit() 
            elif user_commands[0] == "list":
                print map_annotator.list_poses() 
            elif user_commands[0] == "save":
                if len(user_commands) != 2:
                    print "save requires a <name>"
                else:
                    map_annotator.save(user_commands[1])
                    print "saved " + user_commands[1] + " pose"
            elif user_commands[0] == "delete":
                if len(user_commands) != 2:
                    print "delete requires a <name>"
                else: 
                    val = map_annotator.delete(user_commands[1])
                    if val == -1: 
                        print user_commands[1] + " is not a pose"
                    else:
                        print user_commands[1] + " deleted"
            elif user_commands[0] == "goto":
                if len(user_commands) != 2:
                    print "goto requires a <name>"
                else: 
                    val = map_annotator.goto(user_commands[1])
                    if val == -1:
                        print user_commands[1] + " is not a pose"
                    else: 
                        print "going towards " + user_commands[1]
            elif user_commands[0] == "help":
                print msg
            
    except Exception as e:
        rospy.logerr('{}'.format(e))
    finally:
        map_annotator.pickle_dump()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
