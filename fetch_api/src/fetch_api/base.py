#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf.transformations as tft 
import numpy as np
import copy


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    SLEEP_FOR_ODOM = 0.5

    def __init__(self):
        # Create publisher
        self._pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        self._odom_sub = rospy.Subscriber('odom', Odometry, callback=self._odom_callback)
        self._last_odom = None

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
        self._pub.publish(msg)

    def stop(self):
        """Stops the mobile base from moving.
        """
        # Publish 0 velocity
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        self._pub.publish(msg)
    
    def _odom_callback(self, msg):
        self._last_odom = msg

    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        # rospy.sleep until the base has received at least one message on /odom
        while self._last_odom == None:
            rospy.sleep(self.SLEEP_FOR_ODOM)

        # record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self._last_odom)
        rate = rospy.Rate(10)

        # check if the robot has traveled the desired distance, checking for negative distance
        while self.distance_between_points(self._last_odom.pose.pose.position.x, self._last_odom.pose.pose.position.y, start.pose.pose.position.x, start.pose.pose.position.y) < abs(distance):
            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            rate.sleep()
    
s
    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        # rospy.sleep until the base has received at least one message on /odom
        while self._last_odom == None:
            rospy.sleep(self.SLEEP_FOR_ODOM)
        # record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self._last_odom)

        # What will you do if angular_distance is greater than 2*pi or less than -2*pi?
        modified_distance = angular_distance % (2*math.pi) 

        # get back negative rotation if needed
        modified_distance = modified_distance * math.signum(angular_distance)

        rate = rospy.Rate(10)

        # TODO: CONDITION should check if the robot has rotated the desired amount
        # TODO: Be sure to handle the case where the desired amount is negative!


        start_quaternion = start.pose.pose.orientation.quaternion 

        start_angle = angular_position_in_rads(start_quaternion)

        # compute desired yaw, aka "goal angle", given angular_distance 



        current_angle = angular_position_in_rads(self._last_odom.pose.pose.orientation.quaternion)

        remaining_angular_distance = 0

        # if angular_distance - current_angle > 180:
        #     # stuff
        # else: 
        #     # more stuff


        while CONDITION:
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if angular_distance < 0 else 1
            self.move(0, direction * speed)
            rate.sleep()

    # don't worry about z axis because robot travels on flat ground presumably
    def distance_between_points(self, x1, y1, x2, y2):
            sq1 = (x1-x2)*(x1-x2)
            sq2 = (y1-y2)*(y1-y2)
            return math.sqrt(sq1+sq2)
    
    # q is a Quaternion. function computes theta, the rotation about the z axis, in radians 
    def angular_position_in_rads(self, q):
        rotation_matrix = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        x = rotation_matrix[0, 0]
        y = rotation_matrix[1, 0]
        theta_rads = math.atan2(y, x)
        return theta_rads