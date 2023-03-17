#!/usr/bin/env python
# coding: utf-8

"""
Used to get picking images from random position. The images go to 'success/<rgb and depth>' or 'fail/<rgb and depth>' folders, depending if the gripper succeed or not

- We need to establish a connection to the robot with the following command:
roslaunch raiv_libraries ur3_bringup_cartesian.launch robot_ip:=10.31.56.102 kinematics_config:=${HOME}/Calibration/ur3_calibration.yaml

- Information from Arduino
rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0

- launch program:
python random_pick_birdview.py <images_folder> <calibration_files_folder>

"""
import rospy
from pathlib import Path
import sys
import os
from cv_bridge import CvBridge
from raiv_camera_calibration.perspective_calibration import PerspectiveCalibration
from raiv_libraries.robot_with_vaccum_gripper import Robot_with_vaccum_gripper
from raiv_libraries.srv import get_coordservice
from raiv_libraries.get_coord_node import InBoxCoord
from raiv_libraries.image_tools import ImageTools
from raiv_libraries import tools
import argparse

#
# Constants
#
Z_PICK_PLACE = 0.12  # Z coord to start pick or place movement (in meter)
X_INT = 0.3  # XYZ coord where the robot is on intermediaire position (in meter)
Y_INT = 0.0
Z_INT = 0.12
X_OUT = 0.21  # XYZ coord where the robot is out of camera scope (in meter)
Y_OUT = -0.27
Z_OUT = 0.12

def print_info(object_gripped, nb_success, nb_fail, check):
    os.system('clear')
    print('##############################################')
    print()
    print('SUCCESS' if object_gripped else 'FAIL')
    print()
    print(f'Success images : {nb_success}')
    print(f'Fail images    : {nb_fail}')
    print('##############################################')
    print()
    if check:   # If manual check, we ask user if we need to save images
        rep = None
        while rep!='y' and rep!='n':
            rep = input("Do you want to save this sample? ('y' or 'n') : ")
    else:
        rep = 'y'
    return rep

#
# Main program
#
parser = argparse.ArgumentParser(description="Get picking images from random position and put them in '<rgb or depth>'/<'success' or 'fail>' folders")
parser.add_argument('images_folder', type=str, help="images folder for sub-folders 'rgb' and 'depth'")
parser.add_argument('calibration_folder', type=str, help='camera calibration folder')
parser.add_argument('-c', '--check', default=False, action='store_true', help='perform a manual check for each sample (user has to validate if the program saves images')
args = parser.parse_args()

# Create, if they don't exist, <images_folder>/rgb/success, <images_folder>/depth/success,
# <images_folder>/rgb/fail and <images_folder>/depth/fail folders
parent_image_folder = Path(args.images_folder)
tools.create_rgb_depth_folders(parent_image_folder)

rospy.init_node('random_picks_birdview')
# A UR robot with a vaccum gripper
robot = Robot_with_vaccum_gripper()
robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT)  # Go out of camera scope
# We can now ask a service to get and process 3D images
coord_service_name = 'In_box_coordService'
rospy.wait_for_service(coord_service_name)
coord_service = rospy.ServiceProxy(coord_service_name, get_coordservice)
# A PerspectiveCalibration object to perform 2D => 3D conversion
dPoint = PerspectiveCalibration(args.calibration_folder)
bridge = CvBridge()
nb_success = 0
nb_fail = 0
# Main loop to get image
while True:
    # Get all information from the camera
    resp_pick = coord_service('random', InBoxCoord.PICK, InBoxCoord.ON_OBJECT, tools.BIG_CROP_WIDTH, tools.BIG_CROP_HEIGHT, None, None)
    resp_place = coord_service('random', InBoxCoord.PLACE, InBoxCoord.IN_THE_BOX, None, None, None, None)
    # Move robot to pick position
    pick_pose = tools.xyz_to_pose(resp_pick.x_robot, resp_pick.y_robot, Z_PICK_PLACE)
    robot.pick(pick_pose)
    # Place the object
    place_pose = tools.xyz_to_pose(resp_place.x_robot, resp_place.y_robot, Z_PICK_PLACE)
    robot.place(place_pose)
    object_gripped = robot.check_if_object_gripped()  # Test if object is gripped
    save_response = print_info(object_gripped, nb_success, nb_fail, args.check)
    robot.release_gripper()        # Switch off the gripper
    robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT, duration=2)  # The robot must go out of the camera field
    if save_response == 'y':
        nb_images = tools.generate_and_save_rgb_depth_images(resp_pick, parent_image_folder, object_gripped)
        if object_gripped:
            nb_success += nb_images
        else:
            nb_fail += nb_images