#!/usr/bin/env python

import fetch_api
import rospy
from web_teleop.srv import SetTorso, SetTorsoResponse, TiltHead, TiltHeadResponse,AdjustGripper, AdjustGripperResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = fetch_api.Torso()
        self._head = fetch_api.Head()
        self._gripper = fetch_api.Gripper()

    def handle_set_torso(self, request):
        # move the torso to the requested height
        self._torso.set_height(request.height)
        return SetTorsoResponse()
    
    def handle_tilt_head(self, request):
        self._head.pan_tilt(0, request.position)
        return TiltHeadResponse()
    
    def handle_adjust_gripper(self, request):
        self._gripper.move(request.action)
        return AdjustGripperResponse()


def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso,
                                  server.handle_set_torso)
    head_service = rospy.Service('web_teleop/tilt_head', TiltHead, server.handle_tilt_head)
    gripper_service = rospy.Service('web_teleop/adjust_gripper', AdjustGripper, server.handle_adjust_gripper)
    rospy.spin()


if __name__ == '__main__':
    main()