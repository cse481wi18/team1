from geometry_msgs.msg import PoseStamped
import fetch_api
import moveit_commander
import rospy
import sys


class SweeperArm(object):
    def __init__(self):
        self._moveit_commander.roscpp_initialize(sys.argv)
        self._moveit_robot = moveit_commander.RobotCommander()
        self._group = moveit_commander.MoveGroupCommander('arm')

        def on_shutdown():
            self._group.stop()
            moveit_commander.roscpp_shutdown()

        rospy.on_shutdown(on_shutdown)

        self._arm = fetch_api.Arm()

    def hand_broom_pass(height, coords):
        pass

    def swiffer_broom_pass(height, coords):
        pass

    def left_right_pass(orientation, height, coords, stroke_width):
        pass
        