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
import math
from datetime import datetime
from pathlib import Path
import sys
from cv_bridge import CvBridge
import cv2
from raiv_camera_calibration.perspective_calibration import PerspectiveCalibration
from raiv_libraries.robot_with_vaccum_gripper import Robot_with_vaccum_gripper
from raiv_libraries.srv import get_coordservice, get_coordserviceResponse
from raiv_libraries.robotUR import RobotUR
import geometry_msgs.msg as geometry_msgs
from sensor_msgs.msg import Image

#
# Constants
#

CROP_WIDTH = 50 # Width and height for rgb and depth cropped images
CROP_HEIGHT = 50
Z_PICK_PLACE = 0.12  # Z coord to start pick or place movement
X_OUT = 0.0  # XYZ coord where the robot is out of camera scope
Y_OUT = -0.3
Z_OUT = 0.12
bridge = CvBridge()

def normalize(image, bins = 255):
    image_histogram, bins = np.histogram(image.flatten(), bins, density=True)
    cdf = image_histogram.cumsum() # cumulative distribution function
    cdf = cdf / cdf[-1] # normalize

    # use linear interpolation of cdf to find new pixel values
    image_equalized = np.interp(image.flatten(), bins[:-1], cdf)

    return image_equalized.reshape(image.shape), cdf

def pixel_to_pose(px, py):
    """ Transpose pixel coord to XYZ coord (in the base robot frame) and return the corresponding frame """
    xyz = dPoint.from_2d_to_3d([px, py])

    while xyz == [['a'],['b'],['c']]:
        resp = coord_service('random', CROP_WIDTH, CROP_HEIGHT)
        print('Asking for a new couple of coordonates due to false value. IN RANDOM PICK BIRDVIEW')
    x = xyz[0][0] / 100
    y = xyz[1][0] / 100
    return geometry_msgs.Pose(
        geometry_msgs.Vector3(x, y, Z_PICK_PLACE), RobotUR.tool_down_pose
    )

def save_images(folder, rgb, depth):
    image_name = str(datetime.now()) + '.png'
    for image_type, image in zip(['rgb', 'depth'], [resp.rgb, resp.depth]):
        image_path = (parent_image_folder / folder / image_type / image_name).resolve()
        cv2.imwrite(str(image_path), image)
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

rospy.init_node('random_picks_birdview')
# A UR robot with a vaccum gripper
robot = Robot_with_vaccum_gripper()
robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT) # Go out of camera scope
# We can now ask a service to get and process 3D images
coord_service_name = 'In_box_coordService'
rospy.wait_for_service(coord_service_name)
coord_service = rospy.ServiceProxy(coord_service_name, get_coordservice)
_ = coord_service('refresh', CROP_WIDTH, CROP_HEIGHT, 0,0) # ask to refresh (i.e get a new image and find boxes)

# Create, if they don't exist, <images_folder>/success/rgb, <images_folder>/success/depth,  <images_folder>/fail/rgb and <images_folder>/fail/depth folders
parent_image_folder = Path(sys.argv[1])
for sf_folder in ['success','fail']:
    for rd_folder in ['rgb', 'depth']:
        folder = parent_image_folder / sf_folder / rd_folder
        Path.mkdir(folder, parents=True, exist_ok=True)

# A PerspectiveCalibration object to perform 2D => 3D conversion
dPoint = PerspectiveCalibration(calibration_folder)

# Main loop to get image
while True:

    # Get all information from the camera
    resp = coord_service('random', CROP_WIDTH, CROP_HEIGHT, 0, 0)
    print(resp)

    # For debug
    distance = rospy.wait_for_message('/Distance_Here', Image)
    distance = bridge.imgmsg_to_cv2(distance, desired_encoding = 'passthrough')

    # coord_correction(resp.hist_max, resp.xpick, resp.ypick, distance)

    resp.rgb = bridge.imgmsg_to_cv2(resp.rgb, desired_encoding = 'passthrough')
    resp.depth = bridge.imgmsg_to_cv2(resp.depth, desired_encoding = 'passthrough')
    resp.depth = resp.depth.astype(np.uint16)

    resp.depth = normalize(resp.depth)[0]
    resp.depth = resp.depth*255
    cv2.imwrite('/home/student1/Desktop/rgb.png', resp.rgb)
    cv2.imwrite('/home/student1/Desktop/depth.png', resp.depth)

    rgb256 = cv2.resize(resp.rgb, (256,256))
    depth256 = cv2.resize(resp.depth, (256,256))

    cv2.imshow("rgb256", rgb256)
    cv2.imshow("depth256", depth256)

    cv2.waitKey(1000)

    print(resp.xplace, 'Xplace')
    print(resp.yplace, 'Yplace')

    # Move robot to pick position
    pick_pose = pixel_to_pose(resp.xpick, resp.ypick)
    object_gripped = robot.pick(pick_pose)
    # If an object is gripped
    if object_gripped:
        # Place the object
        place_pose = pixel_to_pose(resp.xplace, resp.yplace)
        robot.place(place_pose)
        save_images('success', rgb256, depth256)# Save images in success folders
        robot.go_to_xyz_position(0.3, 0, 0.12, duration=2)#Intermediate position to avoid collision with the shoulder
    else:
        robot._send_gripper_message(False)  # Switch off the gripper
        save_images('fail', rgb256, depth256)# Save images in fail folders
        robot.go_to_xyz_position(0.3, 0, 0.12, duration=2)#Intermediate position to avoid collision with the shoulder
    # The robot must go out of the camera field
    robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT, duration = 2)
