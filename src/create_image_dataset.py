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
from datetime import datetime
from pathlib import Path
import sys
import cv2
from raiv_camera_calibration.perspective_calibration import PerspectiveCalibration
from raiv_libraries.robot_with_vaccum_gripper import Robot_with_vaccum_gripper
from raiv_libraries.simple_image_controller import SimpleImageController
from raiv_libraries.robotUR import RobotUR
import geometry_msgs.msg as geometry_msgs
from PyQt5.QtWidgets import *
from PyQt5 import uic

#
# Constants
#
CROP_WIDTH = 25 # Width and height for rgb and depth cropped images
CROP_HEIGHT = 25
Z_PICK_PLACE = 0.1  # Z coord to start pick or place movement
X_OUT = 0.0  # XYZ coord where the robot is out of camera scope
Y_OUT = -0.3
Z_OUT = 0.1
PLACE_POSE = geometry_msgs.Pose(
            geometry_msgs.Vector3(0.2, 0.3, Z_PICK_PLACE), RobotUR.tool_down_pose   MODIFIER LE X ET LE Y POUR LA ZONE DE DEPOT DE L'OBJET
        )

class CreateImageDataset(QWidget):
    """
    XXXX
    """

    def __init__(self):
        super().__init__()
        uic.loadUi("create_image_dataset.ui",self) #needs the canvas_cerate_image_dataset.py file in the current directory
        # Event handlers
        self.btn_calibration_folder.clicked.connect(self._select_calibration_folder)
        self.btn_image_folder.clicked.connect(self._select_image_folder)
        self.btn_launch_robot.clicked.connect(self._launch_robot)
        # Attributs
        self.robot = None
        self.calibration_folder = None
        self.image_folder = None
        # Load a first image
        self.image_controller = SimpleImageController(image_topic='/usb_cam/image_raw') A REMPLACER PAR UN TOPIC PERMETTANT LA RECUPERATION DES 2 IMAGES RGB et DEPTH
        self._set_image()

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
        parent_image_folder = Path(sys.argv[1])
        for sf_folder in ['success', 'fail']:
            for rd_folder in ['rgb', 'depth']:
                folder = parent_image_folder / sf_folder / rd_folder
                Path.mkdir(folder, parents=True, exist_ok=True)
        self.robot = Robot_with_vaccum_gripper()
        self.robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT) # Send robot out of camera scope
        # A PerspectiveCalibration object to perform 2D => 3D conversion
        self.dPoint = PerspectiveCalibration(self.calibration_folder)

    #
    # Public method
    #
    def process_click(self, px, py):
        """ send the robot to this (px, py) position and store the image file in the good folder (success or fail) """
        # Move robot to pick position
        pick_pose = self._pixel_to_pose(px, py)
        object_gripped = self.robot.pick(pick_pose)
        # If an object is gripped
        if object_gripped:
            # Place the object
            print('Gripped')
            self.robot.place(PLACE_POSE)
            self._save_images('success', self.rgb, self.depth)  # Save images in success folders
        else:
            self.robot._send_gripper_message(False)  # Switch off the gripper
            self._save_images('fail', self.rgb, self.depth)  # Save images in fail folders
        # The robot must go out of the camera field
        self.robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT)
        # Get a new image
        self._set_image()

    #
    # Private methods
    #
    def _set_image(self):
        """ Get an image from topic and display it on the canvas """
        rgb, width, height = self.image_controller.get_image()  A REMPLACER PAR LA RECUPERATION DES 2 IMAGES RGB et DEPTH
        self.canvas.set_image(rgb)
        self.rgb = rgb
        self.depth =

    def _pixel_to_pose(self, px, py):
        """ Transpose pixel coord to XYZ coord (in the base robot frame) and return the corresponding frame """
        xyz = self.dPoint.from_2d_to_3d([px, py])
        x = xyz[0][0] / 100
        y = xyz[1][0] / 100
        return geometry_msgs.Pose(
            geometry_msgs.Vector3(x, y, Z_PICK_PLACE), RobotUR.tool_down_pose
        )

    def _save_images(self, folder, rgb, depth):
        image_name = str(datetime.now()) + '.jpg'
        for image_type, image in zip(['rgb', 'depth'], [rgb, depth]):
            image_path = (self.parent_image_folder / folder / image_type / image_name).resolve()
            cv2.imwrite(str(image_path), image)


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






# Main loop to get image
while True:
    # Move robot to pick position
    pick_pose = pixel_to_pose(resp.xpick, resp.ypick)
    object_gripped = robot.pick(pick_pose)
    # If an object is gripped
    if object_gripped:
        # Place the object
        print('Gripped')
        place_pose = pixel_to_pose(resp.xplace, resp.yplace)
        print()
        robot.place(place_pose)
        save_images('success', resp.rgb, resp.depth)  # Save images in success folders
    else:
        robot._send_gripper_message(False)  # Switch off the gripper
        save_images('fail', resp.rgb, resp.depth)  # Save images in fail folders
    # The robot must go out of the camera field
    robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT)
    #cv2.destroyAllWindows()






