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

X_HIGH_GRIPPED = 0.30
Y_HIGH_GRIPPED = -0.13
Z_HIGH_GRIPPED = 0.064

X_INT_GRIPPED = 0.30
Y_INT_GRIPPED = -0.13
Z_INT_GRIPPED = 0.053

X_LOW_GRIPPED = 0.30
Y_LOW_GRIPPED = -0.13
Z_LOW_GRIPPED = 0.045

X_HIGH_NOT_GRIPPED = 0.30
Y_HIGH_NOT_GRIPPED = 0.13
Z_HIGH_NOT_GRIPPED = 0.064

X_INT_NOT_GRIPPED = 0.30
Y_INT_NOT_GRIPPED = 0.13
Z_INT_NOT_GRIPPED = 0.053

X_LOW_NOT_GRIPPED = 0.30
Y_LOW_NOT_GRIPPED = 0.13
Z_LOW_NOT_GRIPPED = 0.045





if __name__ == '__main__':


    rospy.init_node('test_gripped_object')
    my_robot = Robot_with_vaccum_gripper()
    my_robot.go_to_xyz_position(X_HIGH_GRIPPED, Y_HIGH_GRIPPED, Z_HIGH_GRIPPED)
    print("Press ENTER when the vacuum gripper is 1cm above an object")
    input()
    cmp_gripped = 0
    cmp_not_gripped = 0
    cmp_do_gripped = 0
    cmp_do_not_gripped = 0


    while cmp_do_gripped < 50:
        my_robot.go_to_xyz_position(X_LOW_GRIPPED, Y_LOW_GRIPPED, Z_LOW_GRIPPED)
        my_robot._send_gripper_message(True, timer=1)  # Vaccum gripper ON
        my_robot.go_to_xyz_position(X_INT_GRIPPED, Y_INT_GRIPPED, Z_INT_GRIPPED)
        object_gripped = rospy.wait_for_message('object_gripped',
                                                Bool).data  # Wait until a contact between the gripper and an object
        if object_gripped:
            cmp_gripped += 1
        else:
            cmp_not_gripped += 1

        print(f"Gripped : {cmp_gripped}, Not gripped : {cmp_not_gripped}")
        my_robot._send_gripper_message(False, timer=1)  # Vaccum gripper OFF
        cmp_do_gripped += 1
        print('do compteur : ', cmp_do_gripped)
        my_robot.go_to_xyz_position(X_HIGH_GRIPPED, Y_HIGH_GRIPPED, Z_HIGH_GRIPPED)



    while cmp_do_not_gripped < 50:
        my_robot.go_to_xyz_position(X_LOW_NOT_GRIPPED, Y_LOW_NOT_GRIPPED, Z_LOW_NOT_GRIPPED)
        my_robot._send_gripper_message(True, timer=1)  # Vaccum gripper ON
        my_robot.go_to_xyz_position(X_INT_NOT_GRIPPED, Y_INT_NOT_GRIPPED, Z_INT_NOT_GRIPPED)
        object_gripped = rospy.wait_for_message('object_gripped',
                                                Bool).data  # Wait until a contact between the gripper and an object
        if object_gripped:
            cmp_gripped += 1
        else:
            cmp_not_gripped += 1

        print(f"Gripped : {cmp_gripped}, Not gripped : {cmp_not_gripped}")
        my_robot._send_gripper_message(False, timer=1)  # Vaccum gripper OFF
        cmp_do_not_gripped += 1
        print('do not compteur : ', cmp_do_not_gripped)
        my_robot.go_to_xyz_position(X_HIGH_NOT_GRIPPED, Y_HIGH_NOT_GRIPPED, Z_HIGH_NOT_GRIPPED)

