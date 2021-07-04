#!/usr/bin/env python
# coding: utf-8

import rospy
import copy
from math import pi
from raiv_libraries.robotUR import RobotUR
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    myRobot = RobotUR()
    rospy.init_node('robotUR')
    print("Press ENTER to continue")
    input()
    #myRobot.open_gripper()
    print("Press ENTER to continue")
    input()
    #myRobot.close_gripper()
    # Getting Basic Information
    # ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = myRobot.move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)
    # We can also print the name of the end-effector link for this group:
    eef_link = myRobot.move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)
    # We can get a list of all the groups in the robot:
    group_names = myRobot.robot.get_group_names()
    print("============ Available Planning Groups:", myRobot.robot.get_group_names())
    # Sometimes for debugging it is useful to print the entire state of the robot
    print("============ Printing robot current pose")
    print(myRobot.get_current_pose())
    print("============ Printing robot state")
    print(myRobot.robot.get_current_state())
    print("============ Press `Enter` to execute a movement using a joint state goal ...")
    input()
    # Test of positioning with angular coordinates
    targetReached = myRobot.go_to_joint_state([0, -pi / 4, 0, -pi / 2, 0, pi / 3])
    if targetReached:
        print("Target reached")
    else:
        print("Target not reached")
    print("Press ENTER to continue")
    input()
    # Test of positioning with cartesian coordinates
    print("go_to_pose_goal test")
    pose_goal = Pose()
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    myRobot.go_to_pose_goal(pose_goal)
    print("Press ENTER to continue")
    input()
    # Test of positioning with different cartesian waypoints
    print("exec_cartesian_path test")
    waypoints = []
    wpose = myRobot.get_current_pose().pose
    wpose.position.z += 0.1  # First move up (z)
    wpose.position.y += 0.1  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.x += 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.y -= 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))
    myRobot.exec_cartesian_path(waypoints)
    print("Press ENTER to continue")
    input()
    pose_goal = Pose()
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    myRobot.go_to_pose_goal(pose_goal)
    print("The end")
