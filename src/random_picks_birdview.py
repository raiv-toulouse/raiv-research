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
from raiv_libraries.robotUR import RobotUR
import geometry_msgs.msg as geometry_msgs
from sensor_msgs.msg import Image
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
BIG_CROP_WIDTH = 100  # Crop a big image to be able to perform rotations before final small crop
BIG_CROP_HEIGHT = 100


def normalize(image, bins=255):
    image_histogram, bins = np.histogram(image.flatten(), bins, density=True)
    cdf = image_histogram.cumsum()  # cumulative distribution function
    cdf = cdf / cdf[-1]  # normalize
    # use linear interpolation of cdf to find new pixel values
    image_equalized = np.interp(image.flatten(), bins[:-1], cdf)
    return image_equalized.reshape(image.shape), cdf


def xyz_to_pose(x, y, z):
    return geometry_msgs.Pose(geometry_msgs.Vector3(x, y, z), RobotUR.tool_down_pose)


#
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
    resp_pick = coord_service('random', InBoxCoord.PICK, InBoxCoord.ON_OBJECT, BIG_CROP_WIDTH, BIG_CROP_HEIGHT, None, None)
    resp_place = coord_service('random', InBoxCoord.PLACE, InBoxCoord.IN_THE_BOX, ImageTools.BIG_CROP_WIDTH, ImageTools.BIG_CROP_HEIGHT, None, None)

    # For debug
    distance = rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw', Image)
    distance = bridge.imgmsg_to_cv2(distance, desired_encoding='passthrough')

    # For debug
    #cv2.imshow("rgb256", cv2.resize(rgb_crop, (256, 256)))
    #cv2.imshow("depth256", cv2.resize(depth_crop, (256, 256)))
    #cv2.waitKey(1000)

    # Move robot to pick position
    pick_pose = xyz_to_pose(resp_pick.x_robot, resp_pick.y_robot, Z_PICK_PLACE)
    object_gripped = robot.pick(pick_pose)
    # Place the object
    place_pose = xyz_to_pose(resp_place.x_robot, resp_place.y_robot, Z_PICK_PLACE)
    robot.place(place_pose)
    object_gripped = robot.object_gripped()  # Test if object is gripped
    robot.release_gripper()        # Switch off the gripper

    # Generate images from images returned by service
    # depth_crop = bridge.imgmsg_to_cv2(resp_pick.depth_crop, desired_encoding='passthrough')
    # depth_crop = depth_crop.astype(np.uint16)
    # depth_crop = normalize(depth_crop)[0]
    # depth_crop = depth_crop * 255

    rgb_images_pil = []
    depth_images_pil = []
    pil_rgb = ImageTools.ros_msg_to_pil(resp_pick.rgb_crop)

    depth_crop_cv = bridge.imgmsg_to_cv2(resp_pick.depth_crop, desired_encoding='passthrough')

    # Calculate the histogram of the depth image
    histogram = cv2.calcHist([depth_crop_cv], [0], None, [1000], [1, 1000])
    # Take the index with the maximum values (i.e. the value of the table's distance to the camera) e
    # Every pixel with a value under the table value +BOX_ELEVATION milimeters is set to zero.
    distance_camera_to_table = histogram.argmax()
    image_depth_without_table = np.where(depth_crop_cv == 0, distance_camera_to_table, depth_crop_cv)
    alt_pt_plus_haut = np.min(image_depth_without_table)
    image_depth_without_table = np.where(image_depth_without_table >= distance_camera_to_table - InBoxCoord.THRESHOLD_ABOVE_TABLE, distance_camera_to_table,
                                         image_depth_without_table)

    cv2.normalize(image_depth_without_table, image_depth_without_table, 0, 255, cv2.NORM_MINMAX)
    image_depth_without_table = np.round(image_depth_without_table).astype(np.uint8)
    pil_depth = ImageTools.numpy_to_pil(image_depth_without_table)

    # Generate a set of images with rotation transform
    for deg in range(0, 360, 10):
        rgb_images_pil.append(ImageTools.center_crop(pil_rgb.rotate(deg), ImageTools.BIG_CROP_WIDTH, ImageTools.BIG_CROP_HEIGHT))
        depth_images_pil.append(ImageTools.center_crop(pil_depth.rotate(deg), ImageTools.BIG_CROP_WIDTH, ImageTools.BIG_CROP_HEIGHT))

    tools.save_images(parent_image_folder, 'success' if object_gripped else 'fail', rgb_images_pil, depth_images_pil) # Save images in success folders













    robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT, duration=2)  # The robot must go out of the camera field
