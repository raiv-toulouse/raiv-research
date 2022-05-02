#!/usr/bin/env python
# coding: utf-8

"""
Used to get picking images from selected positions. The images go to 'success/<rgb and depth>' or 'fail/<rgb and depth>' folders,
depending if the gripper succeed or not.

- We need to establish a connection to the robot with the following command:
roslaunch raiv_libraries ur3_bringup_cartesian.launch robot_ip:=10.31.56.102 kinematics_config:=${HOME}/Calibration/ur3_calibration.yaml

- Information from Arduino
rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0

- launch program:
python create_image_dataset.py

"""
import numpy as np
import rospy
from datetime import datetime
from pathlib import Path
import sys
import cv2
from raiv_camera_calibration.perspective_calibration import PerspectiveCalibration
from raiv_libraries.get_coord_node import InBoxCoord
from raiv_libraries.robot_with_vaccum_gripper import Robot_with_vaccum_gripper
from raiv_libraries.srv import get_coordservice
from raiv_libraries.robotUR import RobotUR
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import geometry_msgs.msg as geometry_msgs
from PyQt5.QtWidgets import *
from PyQt5 import uic
import os

#
# Constants
#
CROP_WIDTH = 25 # Width and height for rgb and depth cropped images
CROP_HEIGHT = 25
Z_PICK_PLACE = 0.1  # Z coord to start pick or place movement
X_OUT = 0.0  # XYZ coord where the robot is out of camera scope
Y_OUT = -0.3
Z_OUT = 0.12
PLACE_POSE = geometry_msgs.Pose(
            geometry_msgs.Vector3(0.323, 0.1, Z_PICK_PLACE), RobotUR.tool_down_pose
        )

class CreateImageDataset(QWidget):
    """
    XXXX
    """

    def __init__(self):
        super().__init__()
        uic.loadUi("create_image_dataset.ui",self) #needs the canvas_create_image_dataset.py file in the current directory
        # Event handlers
        self.btn_calibration_folder.clicked.connect(self._select_calibration_folder)
        self.btn_image_folder.clicked.connect(self._select_image_folder)
        self.btn_launch_robot.clicked.connect(self._launch_robot)
        # Attributs
        self.robot = None
        self.calibration_folder = None
        self.image_folder = None

        coord_service_name = 'In_box_coordService'
        rospy.wait_for_service(coord_service_name)
        self.coord_service = rospy.ServiceProxy(coord_service_name, get_coordservice)

        # Load a first image

        bridge = CvBridge()
        self.image_controller = rospy.wait_for_message('/RGBClean', Image)
        # self.image_controller = bridge.imgmsg_to_cv2(self.image_controller, desired_encoding = 'passthrough')
        print(self.image_controller)
        print('Printing image from RGBClean done')
        self.canvas.set_image(self.image_controller)

    #
    # Event handlers
    #
    def _enable_robot_button(self):
        if self.calibration_folder and self.image_folder:
            self.btn_launch_robot.setEnabled(True)

    def _select_calibration_folder(self):
        dir = QFileDialog.getExistingDirectory(self, "Select camera calibration directory", ".",QFileDialog.ShowDirsOnly)
        if dir:
            self.calibration_folder = Path(dir)
            self.lbl_calibration_folder.setText(str(self.calibration_folder))
            self._enable_robot_button()

    def _select_image_folder(self):
        dir = QFileDialog.getExistingDirectory(self, "Select image directory", ".",QFileDialog.ShowDirsOnly)
        if dir:
            self.image_folder = Path(dir)
            self.lbl_image_folder.setText(str(self.image_folder))
            self._enable_robot_button()

    def _launch_robot(self):
        # Create, if they don't exist, <images_folder>/success/rgb, <images_folder>/success/depth,  <images_folder>/fail/rgb and <images_folder>/fail/depth folders
        print(sys.argv)
        print(Path(sys.argv[0]))
        print(self.image_folder)
        self.parent_image_folder = Path(self.image_folder)
        for sf_folder in ['success', 'fail']:
            for rd_folder in ['rgb', 'depth']:
                folder = self.parent_image_folder / sf_folder / rd_folder
                Path.mkdir(folder, parents=True, exist_ok=True)
        self.robot = Robot_with_vaccum_gripper()
        self.robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT) # Send robot out of camera scope
        # A PerspectiveCalibration object to perform 2D => 3D conversion
        self.dPoint = PerspectiveCalibration(self.calibration_folder)
        self.canvas.set_image(rospy.wait_for_message('/RGBClean', Image))


    #
    # Public method
    #
    def process_click(self, px, py):
        """ send the robot to this (px, py) position and store the image file in the good folder (success or fail) """
        self._set_image(px,py)
        # Move robot to pick position
        pick_pose = self._pixel_to_pose(px, py)
        object_gripped = self.robot.pick(pick_pose)
        # If an object is gripped
        if object_gripped:
            # Place the object
            print('Gripped')
            self.robot.place(PLACE_POSE)
            self._save_images('success')  # Save images in success folders
        else:
            self.robot._send_gripper_message(False)  # Switch off the gripper
            self._save_images('fail')  # Save images in fail folders
        # The robot must go out of the camera field
        self.robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT)
        # Get a new image
        self._set_image(px,py)

    #
    # Private methods
    #
    def _set_image(self, px, py):
        bridge = CvBridge()
        """ Get an image from service and display it on the canvas """
        resp = self.coord_service('fixed', InBoxCoord.PICK, InBoxCoord.IN_THE_BOX, CROP_WIDTH, CROP_HEIGHT, px, py)
        self.rgb = resp.rgb_crop
        self.depth = resp.depth_crop
        self.canvas.set_image(rospy.wait_for_message('/RGBClean', Image))
        self.depth = bridge.imgmsg_to_cv2(self.depth, desired_encoding='passthrough')
        self._histeq()

    def _pixel_to_pose(self, px, py):
        """ Transpose pixel coord to XYZ coord (in the base robot frame) and return the corresponding frame """
        x, y, z = self.dPoint.from_2d_to_3d([px, py])
        print('xyz :', x, y, z)
        return geometry_msgs.Pose(
            geometry_msgs.Vector3(x, y, Z_PICK_PLACE), RobotUR.tool_down_pose
        )

    def _save_images(self, folder):
        image_name = str(datetime.now()) + '.png'
        bridge = CvBridge()
        for image_type, image in zip(['rgb', 'depth'], [self.rgb, self.depth]):
            image_path = (self.parent_image_folder / folder / image_type).resolve()
            os.chdir(image_path)
            image = bridge.imgmsg_to_cv2(image, desired_encoding = 'passthrough')
            cv2.imwrite(image_name, image)
            image_name.chmod(0o777)  # Write permission for everybody

    #Funciton used to normalize the image
    def _histeq(self,bins=255):
        bridge = CvBridge()
        image_histogram, bins = np.histogram(self.depth.flatten(), bins, density=True)
        cdf = image_histogram.cumsum()  # cumulative distribution function
        cdf = cdf / cdf[-1]  # normalize

        # use linear interpolation of cdf to find new pixel values
        image_equalized = np.interp(self.depth.flatten(), bins[:-1], cdf)
        image_equalized = image_equalized.reshape(self.depth.shape)
        self.depth = image_equalized*255
        self.depth = bridge.cv2_to_imgmsg(self.depth, encoding = 'passthrough')

#
# Main program
#
if __name__ == '__main__':
    rospy.init_node('create_image_dataset')
    rate = rospy.Rate(0.5)
    app = QApplication(sys.argv)
    gui = CreateImageDataset()
    gui.show()
    sys.exit(app.exec_())