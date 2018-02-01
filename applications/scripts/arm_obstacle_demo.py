#! /usr/bin/env python

import fetch_api
import rospy
from moveit_python import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('arm_obstacle_demo')
    wait_for_time()

    planning_scene = PlanningSceneInterface('base_link')

    # Create table obstacle
    planning_scene.removeCollisionObject('table')
    table_size_x = 0.5
    table_size_y = 1
    table_size_z = 0.03
    table_x = 0.8
    table_y = 0
    table_z = 0.6
    planning_scene.addBox('table', table_size_x, table_size_y, table_size_z,
                        table_x, table_y, table_z)

    # Create divider obstacle
    planning_scene.removeCollisionObject('divider')
    size_x = 0.3 
    size_y = 0.01
    size_z = 0.4 
    x = table_x - (table_size_x / 2) + (size_x / 2)
    y = 0 
    z = table_z + (table_size_z / 2) + (size_z / 2)
    planning_scene.addBox('divider', size_x, size_y, size_z, x, y, z)

    

if __name__ == '__main__':
    main()
