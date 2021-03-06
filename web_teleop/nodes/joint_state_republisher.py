#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Float64MultiArray
from joint_state_reader import JointStateReader

QUEUE_SIZE = 10

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('joint_state_republisher')
    wait_for_time()
    torso_pub = rospy.Publisher('joint_state_republisher/torso_lift_joint',
                                Float64, queue_size = QUEUE_SIZE)
    head_pub = rospy.Publisher('joint_state_republisher/head_tilt_joint',
                                Float64, queue_size = QUEUE_SIZE)      
    gripper_pub = rospy.Publisher('joint_state_republisher/l_gripper_finger_joint',
                                    Float64,queue_size = QUEUE_SIZE)    

    arm_pub = rospy.Publisher('joint_state_republisher/arm_joints',
                                    Float64MultiArray,queue_size = QUEUE_SIZE)                
    reader = JointStateReader()
    rospy.sleep(0.5)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # get torso joint value
        torso_val = reader.get_joint("torso_lift_joint")
        # publish torso joint value
        torso_pub.publish(torso_val)

        head_val = reader.get_joint("head_tilt_joint")
        # publish head joint value
        head_pub.publish(head_val)

        gripper_val = reader.get_joint("l_gripper_finger_joint")
        gripper_pub.publish(gripper_val)

        arm_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        arm_vals = reader.get_joints(arm_joints)
        msg = Float64MultiArray()
        msg.data = arm_vals
        arm_pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    main()