import sys
from PyQt5.QtWidgets import *
from PyQt5 import uic
import time
import matplotlib
import rospy
from raiv_libraries.robotUR import RobotUR
from raiv_libraries.robot_with_vaccum_gripper import Robot_with_vaccum_gripper
from raiv_camera_calibration.perspective_calibration import PerspectiveCalibration
import geometry_msgs.msg as geometry_msgs
from raiv_libraries.image_tools import ImageTools
import PIL.Image
from pathlib import Path
from raiv_libraries.rgb_and_depth_cnn import RgbAndDepthCnn
from sensor_msgs.msg import Image
from PyQt5.QtWidgets import QMessageBox
from raiv_libraries.rgb_cnn import RgbCnn
from raiv_libraries.cnn import Cnn
import os

# global variables
Z_PICK_ROBOT = 0.15  # Z coord before going down to pick
X_OUT = 0.0  # XYZ coord where the robot is out of camera scope
Y_OUT = -0.3
Z_OUT = 0.12

### Used for DEBUG purpose
matplotlib.use('Qt5Agg')


class ExploreWindow(QWidget):
    """
    Load an image and a CNN model from a CKPT file and display the prediction for some sub-images at some specific points
    """

    def __init__(self, calibration_folder, rgb_and_depth):
        super().__init__()
        self.rgb_and_depth = rgb_and_depth  # True : RGB + DEPTH model, False : only RGB model
        uic.loadUi("explore_ihm.ui", self)  # needs the canvas_explore.py file in the current directory
        self.title = 'Camera'
        # event handlers
        self.btn_load_model.clicked.connect(self._load_model)
        self.btn_load_image.clicked.connect(self._load_image)
        self.btn_save_image.clicked.connect(self._save_image)
        self.btn_load_crop_image.clicked.connect(self.predict_from_image)
        self.btn_update_image.clicked.connect(self._set_image)
        self.btn_change_image.clicked.connect(self.move_robot)
        self.btn_activate_robot.clicked.connect(self._activate_robot)
        self.sb_threshold.valueChanged.connect(self._change_threshold)
        self.btn_compute.clicked.connect(self._compute_pred_at_x_y)
        # attributs
        self.dPoint = PerspectiveCalibration(calibration_folder)
        self.default_images_folder = '.'
        #self.image_controller = RgbAndDepthImageController()
        self.rgb_topic = '/camera/color/image_raw'
        self.depth_topic = '/depth_256_image'
        self.model = None
        self.robot = None
        self._set_image()
        self._load_model()
        self.canvas.setup_after_creation()

    def _load_image(self):
        loaded_image = QFileDialog.getOpenFileName(self, 'Open image', self.default_images_folder, "Image files (*.png)",
                                                   options=QFileDialog.DontUseNativeDialog)
        if loaded_image[0]:
            img_pil = PIL.Image.open(loaded_image[0])
            self.image = img_pil
            if self.rgb_and_depth:
                filename_without_ext = os.path.splitext(loaded_image[0])[0]
                depth_pil = PIL.Image.open(filename_without_ext+'_depth.png')
            else:
                depth_pil = None
            self.canvas.set_image(img_pil, depth_pil)

    def _save_image(self):
        filename = QFileDialog.getSaveFileName(self, 'Save image to file', '.', "Image files (*.png)",
                                                   options=QFileDialog.DontUseNativeDialog)
        self.image.save(filename[0]+'.png', 'png')
        if self.rgb_and_depth:
            self.depth_image.save(filename[0]+'_depth.png', 'png')

    def _compute_pred_at_x_y(self):
        self.canvas.compute_pred_at_x_y()

    def predict_from_point(self, x, y):
        """ Predict probability and class for a cropped image at (x,y) """
        self.predict_center_x = x
        self.predict_center_y = y
        rgb_crop_pil = ImageTools.crop_xy(self.image, x, y, ImageTools.CROP_WIDTH, ImageTools.CROP_HEIGHT)
        if self.rgb_and_depth:
            depth_crop_pil = ImageTools.crop_xy(self.depth_image, x, y, ImageTools.CROP_WIDTH, ImageTools.CROP_HEIGHT)
            return RgbAndDepthCnn.predict_from_pil_rgb_and_depth_images(self.model, rgb_crop_pil, depth_crop_pil)
        else:
            return RgbCnn.predict_from_pil_rgb_image(self.model, rgb_crop_pil)

    def predict_from_image(self):
        """ Load the images data """
        loaded_image = QFileDialog.getOpenFileName(self, 'Open cropped image', self.default_images_folder, "Image files (*.png *.jpg)",
                                                   options=QFileDialog.DontUseNativeDialog)
        self.default_images_folder = os.path.dirname(loaded_image[0])
        if loaded_image[0]:
            image_pil = PIL.Image.open(loaded_image[0])
            pred = RgbCnn.predict_from_pil_rgb_image(self.model, image_pil)
            prob, cl = Cnn.compute_prob_and_class(pred)
            self.lbl_result_map.setText(f"The prediction for this image is : {prob*100:.2f}%" )

    def compute_map(self, start_coord, end_coord):
        """ Compute a list of predictions and ask the canvas to draw them
            Called from CanvasExplore """
        all_preds = self._compute_all_preds(start_coord, end_coord)
        self.canvas.all_preds = all_preds
        self.canvas.repaint()

    def ask_robot_to_pick(self, px, py):
        if self.robot:
            x, y, z = self.dPoint.from_2d_to_3d([px, py])
            print("Pixel coord = {:.0f}, {:.0f}".format(px, py))
            pose_for_pick = geometry_msgs.Pose(
                geometry_msgs.Vector3(x, y, Z_PICK_ROBOT), RobotUR.tool_down_pose
            )
            self.robot.pick(pose_for_pick)
            self.robot.release_gripper()

    ############ Private methods ################

    # Event handler

    def _change_threshold(self):
        ''' Redraw the predictions if the threshold has been changed '''
        self.canvas.repaint()

    def _load_model(self):
        """ Load a new model """
        fname = QFileDialog.getOpenFileName(self, 'Open CKPT model file', '.', "Model files (*.ckpt)",
                                            options=QFileDialog.DontUseNativeDialog)
        if fname[0]:
            if self.rgb_and_depth: # model for RGB and DEPTH images
                self.model = RgbAndDepthCnn.load_ckpt_model_file(fname[0])
            else:  # model for only RGB images
                self.model = RgbCnn.load_ckpt_model_file(fname[0])  # Load the selected models
            ckpt_model_name = os.path.basename(fname[0])  # Only the name, without path
            self.lbl_model_name.setText(ckpt_model_name)

    def _activate_robot(self):
        """ """
        self.robot = Robot_with_vaccum_gripper()
        self.btn_change_image.setEnabled(True)

    def move_robot(self):
        """  Move robot out of camera scope then get and display a new image """
        if self.robot:
            self.robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT, duration=4)
            self._set_image()
        else:
            QMessageBox.warning(self, "No robot", "Don't forget to initialize the robot")

    def _set_image(self):
        """ Get an image from topic and display it on the canvas """
        msg_rgb = rospy.wait_for_message(self.rgb_topic, Image)
        msg_depth = rospy.wait_for_message(self.depth_topic, Image)
        pil_rgb = ImageTools.ros_msg_to_pil(msg_rgb)
        pil_depth = ImageTools.ros_msg_to_pil(msg_depth)
        self.canvas.set_image(pil_rgb, pil_depth)
        self.image = pil_rgb
        self.depth_image = pil_depth

    def _compute_all_preds(self, start_coord, end_coord):
        """ Compute a list of predictions like :
        [ [x, y, tensor([[prob_fail, proba_success]])], ...] with x,y the center of cropped image size (WIDTH,HEIGHT)
        """
        start = time.time()
        all_preds = []
        steps = int(self.edt_nb_pixels_per_step.text())
        count = 0
        for x in range(start_coord.x(), end_coord.x(), steps):
            for y in range(start_coord.y(), end_coord.y(), steps):
                preds = self.predict_from_point(x, y)
                all_preds.append([x, y, preds])
                count += 1
        end = time.time()
        self.lbl_result_map.setText(f'{count} inferences in {end - start:.1f} s')
        return all_preds


# First, run the communication between the robot and ROS :
# roslaunch raiv_libraries ur3_bringup_cartesian.launch robot_ip:=10.31.56.102 kinematics_config:=${HOME}/Calibration/ur3_calibration.yaml
# rosrun usb_cam usb_cam_node _image_width:=1280 _image_height:=960 >/dev/null 2>&1
# Then, run this node :
# python explore.py <camera_calibration_folder>

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Test a CKPT file model and perform robot pick action.')
    parser.add_argument('calibration_folder', type=str, help='calibration files folder')
    parser.add_argument('--rgb_and_depth', default=False, action='store_true', help='For RGB + DEPTH model')
    args = parser.parse_args()

    rospy.init_node('explore')
    rate = rospy.Rate(0.5)
    app = QApplication(sys.argv)
    gui = ExploreWindow(args.calibration_folder, args.rgb_and_depth)
    gui.show()
    sys.exit(app.exec_())
