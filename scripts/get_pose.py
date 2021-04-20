#!/usr/bin/env python

import sys
import rospy
import moveit_commander

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_get_pose_demo', anonymous=True)

    robot = moveit_commander.RobotCommander()

    rospy.sleep(1)

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    print("Current state:")
    print(robot.get_current_state())

    while True:
        a = raw_input()
        if a == "q":
            break

        print(move_group.get_current_pose())