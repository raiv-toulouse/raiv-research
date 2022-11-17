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
from raiv_libraries.get_coord_node import InBoxCoord
from raiv_libraries import tools
from sensor_msgs.msg import Image
from raiv_libraries.srv import get_coordservice
from cv_bridge import CvBridge
from PyQt5.QtWidgets import *
from PyQt5 import uic
import os

#
# Constants
#
Z_PICK_PLACE = 0.1  # Z coord to start pick or place movement
X_OUT = 0.0  # XYZ coord where the robot is out of camera scope
Y_OUT = -0.3
Z_OUT = 0.12


class CreateFakeDataset(QWidget):
    """
    Ne pas oublier de lancer avant :
    * roslaunch realsense2_camera rs_camera.launch align_depth:=true
    * rosrun raiv_libraries get_coord_node.py

    Permet de créer une banque d'images en cliquant sur l'image (pas de mouvement du robot)
    """

    def __init__(self):
        super().__init__()
        uic.loadUi("create_fake_dataset.ui",self) #needs the canvas_create_image_dataset.py file in the current directory
        # Event handlers
        self.btn_image_folder.clicked.connect(self._select_image_folder)
        # Attributs
        self.image_folder = None
        # Load a first image
        self.image_controller = rospy.wait_for_message('/camera/color/image_raw', Image)
        coord_service_name = 'In_box_coordService'
        rospy.wait_for_service(coord_service_name)
        self.coord_service = rospy.ServiceProxy(coord_service_name, get_coordservice)
        self.canvas.set_image(self.image_controller)

    #
    # Event handlers
    #

    def _select_image_folder(self):
        dir = QFileDialog.getExistingDirectory(self, "Select image directory", "/common/work/stockage_banque_image/0_5_soufflet", QFileDialog.ShowDirsOnly)
        if dir:
            self.image_folder = Path(dir)
            tools.create_rgb_depth_folders(self.image_folder)
            self.lbl_image_folder.setText(str(self.image_folder))

    #
    # Public method
    #
    def process_click(self, px, py):
        """ store the image file in the good folder (success or fail) """
        self._set_image(px,py)
        # If an object is gripped
        if self.rb_success.isChecked():
            # Place the object
            self._save_images('success')  # Save images in success folders
        else:
            self._save_images('fail')  # Save images in fail folders

    #
    # Private methods
    #
    def _set_image(self, px, py):
        bridge = CvBridge()
        """ Get an image from service and display it on the canvas """
        resp = self.coord_service('fixed', InBoxCoord.PICK, InBoxCoord.IN_THE_BOX, InBoxCoord.CROP_WIDTH, InBoxCoord.CROP_HEIGHT, px, py)
        self.rgb = resp.rgb_crop
        self.depth = resp.depth_crop
        self.canvas.set_image(rospy.wait_for_message('/camera/color/image_raw', Image))

    def _save_images(self, folder):
        image_name = str(datetime.now()) + '.png'
        bridge = CvBridge()
        for image_type, image in zip(['rgb', 'depth'], [self.rgb, self.depth]):
            image_path = (self.image_folder / image_type / folder).resolve()
            os.chdir(image_path)
            image = bridge.imgmsg_to_cv2(image, desired_encoding = 'passthrough')
            cv2.imwrite(image_name, image)
            image_path.chmod(0o777)  # Write permission for everybody

#
# Main program
#
if __name__ == '__main__':
    rospy.init_node('create_fake_dataset')
    rate = rospy.Rate(0.5)
    app = QApplication(sys.argv)
    gui = CreateFakeDataset()
    gui.show()
    sys.exit(app.exec_())