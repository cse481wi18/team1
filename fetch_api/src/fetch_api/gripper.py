#! /usr/bin/env python

# TODO: import ?????????
import control_msgs.msg
import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

# TODO: ACTION_NAME = ???
CLOSED_POS = 0.0  # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).


class Gripper(object):
    """Gripper controls the robot's gripper.
    """
    MIN_EFFORT = 35  # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons

    def __init__(self):
        # TODO: Create actionlib self.client
        # TODO: Wait for server
        self.client = actionlib.SimpleActionClient('gripper_controller/gripper_action', GripperCommandAction)
        self.client.wait_for_server()

    def open(self):
        """Opens the gripper.
        """
        # TODO: Create goal
        # TODO: Send goal
        # TODO: Wait for result
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = OPENED_POS
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return self.client.get_result()
        


    def close(self, max_effort=MAX_EFFORT):
        """Closes the gripper.

        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
        # TODO: Create goal
        # TODO: Send goal
        # TODO: Wait for result
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = CLOSED_POS
        goal.command.max_effort = max_effort

        self.client.send_goal(goal)
        self.client.wait_for_result()
        return self.client.get_result()

    def move(self, action):
        print "IN move gripper"
        if action == CLOSED_POS:
            print "close gripper"
            return self.close()
        elif action == OPENED_POS:
            print "open gripper"
            return self.open()
        print "none:("
        return
