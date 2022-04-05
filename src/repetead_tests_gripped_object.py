#!/usr/bin/env python
# coding: utf-8

# This program is used to test the /gripped_object topic, to check if its measures stay constant along time
# The purpose is :
# 1/ Put the robot arm with its vacuum gripper 1cm above ean object
# 2/ repeat
#   * go down until contact
#   * switch on the vacuum gripper
#   * go up 1cm
#   * read the /gripped_object to test if contact or not and update a counter
#
# rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0
#
# roslaunch raiv_libraries ur3_bringup_cartesian.launch robot_ip:=10.31.56.102 kinematics_config:=/common/calibration/robot/ur3_calibration.yaml
#
# rosrun raiv_research repeated_tests_gripped_contact
#

import rospy
from raiv_libraries.robot_with_vaccum_gripper import Robot_with_vaccum_gripper
from std_msgs.msg import Bool

if __name__ == '__main__':
    rospy.init_node('test_gripped_object')
    my_robot = Robot_with_vaccum_gripper()
    print("Press ENTER when the vacuum gripper is 1cm above an object")
    input()
    cmp_gripped = 0
    cmp_not_gripped = 0
    while True:
        communication_problem = my_robot._down_movement(movement_duration=10)
        if not communication_problem:
            my_robot._send_gripper_message(True, timer=1)   # Vaccum gripper ON
            my_robot._back_to_previous_z()  # Back to the original z pose (go up)
            object_gripped = rospy.wait_for_message('object_gripped',Bool).data  # Wait until a contact between the gripper and an object
            if object_gripped:
                cmp_gripped += 1
            else:
                cmp_not_gripped += 1
            print(f"Gripped : {cmp_gripped}, Not gripped : {cmp_not_gripped}")
            my_robot._send_gripper_message(False, timer=1)   # Vaccum gripper OFF

