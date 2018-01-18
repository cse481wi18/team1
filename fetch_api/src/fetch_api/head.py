#!/usr/bin/env python

import actionlib
import math
import rospy
import control_msgs.msg
import geometry_msgs.msg
import std_msgs.msg 

from control_msgs.msg import PointHeadGoal, PointHeadAction
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header


LOOK_AT_ACTION_NAME = ''  # Get the name of the look-at action
PAN_TILT_ACTION_NAME = ''  # Get the name of the pan/tilt action
PAN_JOINT = 'head_pan_joint'  # Get the name of the head pan joint
TILT_JOINT = 'head_tilt_joint'  # Get the name of the head tilt joint
PAN_TILT_TIME = 2.5  # How many seconds it should take to move the head.

MIN_DURATION = 1
MAX_VELOCITY = 1.57 #rads/sec
class Head(object):
    """Head controls the Fetch's head.

    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians

    For example:
        head = fetch_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """
    MIN_PAN = -math.pi/2  
    MAX_PAN = math.pi/2  
    MIN_TILT = -math.pi/4 
    MAX_TILT = math.pi/2  

    def __init__(self):
        self.client = actionlib.SimpleActionClient('head_controller/point_head', PointHeadAction)
        self.client.wait_for_server()

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.

        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """

        # Create and fill out header (contained in PointStamp msg)
        header = Header()
        header.frame_id = frame_id

        # Create and fill out target point (contained in PointheadGoal)
        pointStamped = PointStamped()
        pointStamped.point = Point(x, y, z)
        pointStamped.header = header
        
        # Create and fill out goal
        goal = control_msgs.msg.PointHeadGoal()
        goal.max_velocity = MAX_VELOCITY
        goal.min_duration = rospy.Duration(MIN_DURATION)
        goal.target = pointStamped
        
        # send goal and wait for result
        self.client.send_goal(goal)
        self.client.wait_for_result()
        
        return self.client.get_result()

    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.

              Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        # TODO: Check that the pan/tilt angles are within joint limits
        # TODO: Create a trajectory point
        # TODO: Set positions of the two joints in the trajectory point
        # TODO: Set time of the trajectory point

        # TODO: Create goal
        # TODO: Add joint names to the list
        # TODO: Add trajectory point created above to trajectory

        # TODO: Send the goal
        # TODO: Wait for result

        rospy.logerr('Not implemented.')