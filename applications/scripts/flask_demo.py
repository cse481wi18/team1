#! /usr/bin/env python
import rospy
from std_msgs.msg import Int64
import time


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def callback(data):
    rospy.loginfo('Received data: %d', data.data)

def listener():
    rospy.init_node('flask_listener_demo')
    wait_for_time()
    rospy.Subscriber('web_interface', Int64, callback)
    rospy.spin()