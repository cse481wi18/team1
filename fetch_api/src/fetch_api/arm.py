#!/usr/bin/env python

import actionlib
import trajectory_msgs.msg
import control_msgs.msg

import rospy

from .arm_joints import ArmJoints
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

TRAJECTORY_TIME = 5

class Arm(object):
    """Arm controls the robot's arm.
    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = fetch_api.Arm()
        arm.move_to_joints(joints)
    """


    def __init__(self):
        # TODO: Create actionlib client
        self.client  = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # TODO: Wait for server
        self.client.wait_for_server()
        

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.
        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # Create a trajectory point
        trajectory_point = JointTrajectoryPoint()

        # Set position of trajectory point
        trajectory_point.positions = arm_joints.values() # array of float positions

        # Set time of trajectory point
        trajectory_point.time_from_start = rospy.Duration(TRAJECTORY_TIME)

        # Trajectory
        trajectory = JointTrajectory()

        # Add joint name to list
        trajectory.joint_names = arm_joints.names()

        #  Add the trajectory point created above to trajectory
        trajectory.points.append(trajectory_point)
  
        # Create goal
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory = trajectory

        # Send goal
        self.client.send_goal(goal)

        # Wait for result
        self.client.wait_for_result()
