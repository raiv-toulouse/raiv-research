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
import numpy as np
import rospy
from pathlib import Path
import sys
import cv2
from cv_bridge import CvBridge
from raiv_camera_calibration.perspective_calibration import PerspectiveCalibration
from raiv_libraries.robot_with_vaccum_gripper import Robot_with_vaccum_gripper
from raiv_libraries.srv import get_coordservice
from raiv_libraries.get_coord_node import InBoxCoord
from raiv_libraries.image_tools import ImageTools
from raiv_libraries import tools

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



def normalize(image, bins=255):
    image_histogram, bins = np.histogram(image.flatten(), bins, density=True)
    cdf = image_histogram.cumsum()  # cumulative distribution function
    cdf = cdf / cdf[-1]  # normalize
    # use linear interpolation of cdf to find new pixel values
    image_equalized = np.interp(image.flatten(), bins[:-1], cdf)
    return image_equalized.reshape(image.shape), cdf


#from raiv_libraries.robotUR import RobotUR
import geometry_msgs.msg as geometry_msgs
from sensor_msgs.msg import Image
# Main program
#

# Check if number of arguments is OK
if len(sys.argv) != 3:
    print("Syntax : python random_picks_birdview.py <images_folder> <calibration_files_folder>")
    exit(1)

# Check if the <calibration_files_folder>" exists
calibration_folder = Path(sys.argv[2])
if not calibration_folder.exists():
    print("This folder doesn't exist : {}".format(sys.argv[2]))
    exit(2)

# Create, if they don't exist, <images_folder>/rgb/success, <images_folder>/depth/success,
# <images_folder>/rgb/fail and <images_folder>/depth/fail folders
parent_image_folder = Path(sys.argv[1])
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
dPoint = PerspectiveCalibration(calibration_folder)
bridge = CvBridge()
# Main loop to get image
while True:

    # Get all information from the camera
    resp_pick = coord_service('random', InBoxCoord.PICK, InBoxCoord.ON_OBJECT, tools.BIG_CROP_WIDTH, tools.BIG_CROP_HEIGHT, None, None)
    resp_place = coord_service('random', InBoxCoord.PLACE, InBoxCoord.IN_THE_BOX, ImageTools.CROP_WIDTH, ImageTools.CROP_HEIGHT, None, None)

    # Move robot to pick position
    pick_pose = tools.xyz_to_pose(resp_pick.x_robot, resp_pick.y_robot, Z_PICK_PLACE)
    robot.pick(pick_pose)
    # Place the object
    place_pose = tools.xyz_to_pose(resp_place.x_robot, resp_place.y_robot, Z_PICK_PLACE)
    robot.place(place_pose)
    object_gripped = robot.check_if_object_gripped()  # Test if object is gripped
    robot.release_gripper()        # Switch off the gripper
    robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT, duration=2)  # The robot must go out of the camera field

    tools.generate_and_save_rgb_depth_images(resp_pick, parent_image_folder, object_gripped)