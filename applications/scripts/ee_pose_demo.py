#! /usr/bin/env python

import fetch_api
import rospy
import time
import math
import tf
import geometry_msgs.msg



def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('tf_pose_listener')
    wait_for_time()

    listener = tf.TransformListener()
    rospy.sleep(0.1) # let TransformListener accumulate messages after creation

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        try:
            # pose of gripper in the base link frame?
            (position, quaternion) = listener.lookupTransform('base_link', 'gripper_link', rospy.Time(0))
            print str(position) + " " + str(quaternion)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()



if __name__ == '__main__':
    main()