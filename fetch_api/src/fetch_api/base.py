#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        # Create publisher
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        # Create Twist msg
        # Fill out msg
        # Publish msg
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        self.pub.publish(msg)

    def stop(self):
        """Stops the mobile base from moving.
        """
        # Publish 0 velocity
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        self.pub.publish(msg)

