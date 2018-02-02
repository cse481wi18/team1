#! /usr/bin/env python

import math
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import ColorRGBA
import visualization_msgs.msg
import rospy
import tf.transformations as tft


def main():
    p_object_in_base_link = Pose()
    p_object_in_base_link.position = Point(0.6, -0.1, 0.7)
    p_object_in_base_link.orientation = Quaternion(0, 0, 0.38268343, 0.92387953)

    t_object_in_base_link = tft.quaternion_matrix(p_object_in_base_link.orientation) # Takes in a plain list of numbers.

if __name__ == '__main__':
    main()