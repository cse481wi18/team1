#! /usr/bin/env python

import fetch_api
import rospy
import time


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('arm_demo')
    wait_for_time()
    argv = rospy.myargv()
    # DISCO_POSES = [[1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0],
    #                [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
    #                [-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
    #                [-1.5, 1.1, -3.0, -0.5, -3.0, -1.0, -3.0],
    #                [-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
    #                [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
    #                [1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]]


    # arm = fetch_api.Arm()
    # for vals in DISCO_POSES:
    #     arm.move_to_joints(fetch_api.ArmJoints.from_list(vals))
    #     time.sleep(1)

    vals = [1.02551778819, -0.580883597877, -1.8110081029,1.83004611882, -0.71210059167, 0.803167709198, -1.46482672643]
    arm = fetch_api.Arm()
    arm.move_to_joints(fetch_api.ArmJoints.from_list(vals))

if __name__ == '__main__':
    main()
