#!/usr/bin/env python

import actionlib
import math
import rospy
import control_msgs.msg
import geometry_msgs.msg
import std_msgs.msg 
import trajectory_msgs.msg

from control_msgs.msg import PointHeadGoal, PointHeadAction
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory



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
        self.point_client = actionlib.SimpleActionClient('head_controller/point_head', PointHeadAction)
        self.point_client.wait_for_server()

        self.trajectory_client = actionlib.SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.trajectory_client.wait_for_server()

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
        self.point_client.send_goal(goal)
        self.point_client.wait_for_result()

        return self.point_client.get_result()

    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.

              Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        # Check that the pan/tilt angles are within joint limits
        if pan < self.MIN_PAN or pan > self.MAX_PAN or tilt < self.MIN_TILT or tilt > self.MAX_TILT:
            return


        # Create a trajectory point (contained in JointTrajectory msg)
        destination = JointTrajectoryPoint()
        # Set position of trajectory point
        destination.positions.append(pan)
        destination.positions.append(tilt)
        # Set time of trajectory point
        destination.time_from_start = rospy.Duration(5)
        
        # Create a joint trajectory (contained in JointTrajectoryGoal)
        trajectory = JointTrajectory()
        # Add joint name to list
        trajectory.joint_names.append(PAN_JOINT)
        trajectory.joint_names.append(TILT_JOINT)
         # Add the trajectory point created above to trajectory
        trajectory.points.append(destination)

        # Create goal
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        # Send goal
        self.trajectory_client.send_goal(goal)
        # Wait for result
        self.trajectory_client.wait_for_result()

        return self.trajectory_client.get_result()

   
