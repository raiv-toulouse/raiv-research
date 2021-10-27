#!/usr/bin/env python
# coding: utf-8

"""
Used to get picking images from random position. The images go to 'success' or 'fail'  folders, depending if the gripper succeed or not

- We need to establish a connection to the robot with the following comand:
roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=10.31.56.102 kinematics_config:=${HOME}/Calibration/ur3_calibration.yaml

- Then, we need to activate moveit server:
roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch

- Information from Arduino
rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0

- launch program:
python random_pick_birdview.py (in robot_controller/src)

"""
from PIL import Image as PILImage
from sensor_msgs.msg import Image
import rospy
from datetime import datetime
import random
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import cv2
import os
from raiv_camera_calibration.perspective_calibration import PerspectiveCalibration
from raiv_libraries.robot_with_vaccum_gripper import Robot_with_vaccum_gripper
from raiv_libraries.simple_image_controller import SimpleImageController
from raiv_libraries.environment import Env_cam_bas
from pathlib import Path
import sys

CROP_WIDTH = 224
CROP_HEIGHT = 224
HALF_CROP_WIDTH = int(CROP_WIDTH / 2)
HALF_CROP_HEIGHT = int(CROP_HEIGHT / 2)

def goto_pixel_position(px, py):
    # transposition des coordonnées pixel du point en coordonnées réelles dans le repère du robot
    xyz = dPoint.from_2d_to_3d([px, py])

    # mouvement vers le point cible
    x = xyz[0][0] / 100
    y = xyz[1][0] / 100
    robot.go_to_xyz_position(x, y, 0.3)

#
# Main program
#
if len(sys.argv) != 3:
    print("Syntax : python random_picks_birdview.py <images_folder> <calibration_files_folder>")
    exit(1)

# Check if the <calibration_files_folder>" exists
calibration_folder = Path(sys.argv[2])
if not calibration_folder.exists():
    print("This folder doesn't exist : {}".format(sys.argv[2]))
    exit(2)

# Create, if they don't exist, <images_folder>/success and <images_folder>/fail
parent_image_folder = Path(sys.argv[1])
success_image_folder = parent_image_folder / 'success'
Path.mkdir(success_image_folder, parents=True, exist_ok=True)
fail_image_folder = parent_image_folder / 'fail'
Path.mkdir(fail_image_folder, parents=True, exist_ok=True)

# Get an image from /usb_cam/image_raw topic
image_controller = SimpleImageController()

# A PrepectiveCalibration object to perform 2D => 3D conversion
dPoint = PerspectiveCalibration(calibration_folder)

# A UR robot with a vaccum gripper
robot = Robot_with_vaccum_gripper(Env_cam_bas())

# initialisation du noeud robotUr
rospy.init_node('random_picks_birdview')

# on remonte de 10cm au cas ou le robot serait trop bas
robot.go_to_initial_position(3)

# boucle du programme de prise d'image
while True:

    # on prend la photo
    img, width, height = image_controller.get_image()

    # Get random pixel point in the box
    px, py = get_random_point(boite_prise)

    # réalisation du crop
    crop = img[py - HALF_CROP_HEIGHT : py + HALF_CROP_HEIGHT, px - HALF_CROP_WIDTH : px + HALF_CROP_WIDTH]

    # For debug
    cv2.imshow("crop", crop)
    cv2.waitKey(1000)

    goto_pixel_position(px, py)  # Move robot to pick position

    # Lancement de l'action de prise
    object_gripped = robot.pick(no_rotation=True)

    # Si un objet est attrapé
    if object_gripped:

        # création d'un point de lâcher aléatoire
        px, py = get_random_point(boite_pose)


        # on éteind la pompe
        robot.send_gripper_message(False)

        # on sauvegarde l'image crop dans le dossier success
        image_path = (success_image_folder / ('success' + str(datetime.now()) + '.jpg')).resolve()
        cv2.imwrite(str(image_path), crop)

    else:

        # on éteind la pompe
        robot.send_gripper_message(False)

        # on sauvegarde l'image crop dans le dossier fail
        image_path = (fail_image_folder / ('fail' + str(datetime.now()) + '.jpg')).resolve()
        cv2.imwrite(str(image_path), crop)

    print("target reached")

    # The robot must go out of the camera field
    robot.go_to_initial_position(3)

    cv2.destroyAllWindows()






