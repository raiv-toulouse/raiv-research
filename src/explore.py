import sys
import cv2
from PyQt5.QtWidgets import *
from PyQt5 import uic
from torchvision.transforms.functional import crop
from torchvision import transforms
import os
import time
import torch
import torchvision
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import rospy
from raiv_libraries.robotUR import RobotUR
from raiv_libraries.robot_with_vaccum_gripper import Robot_with_vaccum_gripper
from raiv_libraries.simple_image_controller import SimpleImageController
from raiv_libraries.image_model import ImageModel
from raiv_camera_calibration.perspective_calibration import PerspectiveCalibration
import geometry_msgs.msg as geometry_msgs


# global variables
WIDTH = HEIGHT = 56 # Size of cropped image
Z_PICK_ROBOT = 0.15 # Z coord before going down to pick

### Used for DEBUG purpose
matplotlib.use('Qt5Agg')

def imshow(images, title=None, pil_image = False):
    """Imshow for Tensor. """
    if pil_image:
        inp = images
    else:
        img_grid = torchvision.utils.make_grid(images).cpu()
        inp = img_grid.numpy().transpose((1, 2, 0))
    mean = np.array([0.485, 0.456, 0.406])
    std = np.array([0.229, 0.224, 0.225])
    inp = std * inp + mean
    inp = np.clip(inp, 0, 1)
    plt.imshow(inp)
    if title is not None:
        plt.title(title)
    plt.pause(2)


class ExploreWindow(QWidget):
    """
    Load an image and a CNN model from a CKPT file and display the prediction for some sub-images at some specific points
    """

    def __init__(self, calibration_folder):
        super().__init__()
        uic.loadUi("explore_ihm.ui",self) #needs the canvas_explore.py file in the current directory
        self.title = 'Camera'
        # event handlers
        self.btn_load_model.clicked.connect(self.load_model)
        self.btn_change_image.clicked.connect(self.move_robot)
        self.sb_threshold.valueChanged.connect(self.change_threshold)
        # attributs
        self.transform = transforms.Compose([
            transforms.Lambda(lambda img: self._crop_xy(img)),
            transforms.Resize(size=256),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        self.dPoint = PerspectiveCalibration(calibration_folder)
        self.image_controller = SimpleImageController(image_topic='/usb_cam/image_raw')
        self.image_model = None
        self.inference_model = None
        self.robot = RobotUR()
        self.move_robot()
        self.load_model()

    def load_model(self):
        """ Load a new model """
        fname = QFileDialog.getOpenFileName(self, 'Open CKPT model file', '.', "Model files (*.ckpt)", options=QFileDialog.DontUseNativeDialog)
        if fname[0]:
            ckpt_model_name = os.path.basename(fname[0])  # Only the name, without path
            self.image_model = ImageModel(model_name='resnet18', ckpt_dir=os.path.dirname(fname[0]))
            self.inference_model = self.image_model.load_ckpt_model_file(ckpt_model_name)  # Load the selected models
            self.lbl_model_name.setText(ckpt_model_name)

    def move_robot(self):
        """  Move robot out of camera scope then get and display a new image """
        # self.robot.go_to_initial_position()
        self._set_image()

    def predict(self, x, y):
        """ Predict probability and class for a cropped image at (x,y) """
        self.predict_center_x = x
        self.predict_center_y = y
        img = self.transform(self.image)  # Get the cropped transformed image
        # imshow(img)  # For DEBUG
        img = img.unsqueeze(0)  # To have a 4-dim tensor ([nb_of_images, channels, w, h])
        features, preds = self.image_model.evaluate_image(img, False)  # No processing
        return torch.exp(preds)

    def compute_map(self, start_coord, end_coord):
        """ Compute a list of predictions and ask the canvas to draw them
            Called from CanvasExplore """
        all_preds = self._compute_all_preds(start_coord, end_coord)
        self.canvas.all_preds = all_preds
        self.canvas.repaint()

    def ask_robot_to_pick(self, px, py):
        xyz = self.dPoint.from_2d_to_3d([px, py])
        print("Pixel coord = {:.0f}, {:.0f}".format(px, py))
        print("XYZ = {:.2f}, {:.2f}, {:.2f}".format(xyz[0][0], xyz[1][0], xyz[2][0]))
        x = xyz[0][0] / 100
        y = xyz[1][0] / 100
        pose_for_pick = geometry_msgs.Pose(
            geometry_msgs.Vector3(x, y, Z_PICK_ROBOT), RobotUR.tool_down_pose
        )
        self.robot.pick(pose_for_pick)

    def change_threshold(self):
        ''' Redraw the predictions if the threshold has been changed '''
        self.canvas.repaint()

    ############ Private methods ################

    @torch.no_grad()
    def _evaluate_image(self, image):
        features, prediction = self.inference_model(image)
        return features.detach().numpy(), prediction.detach()

    def _set_image(self):
        """ Get an image from topic and display it on the canvas """
        img, width, height = self.image_controller.get_image()
        self.canvas.set_image(img)
        self.image = img

    def _crop_xy(self, image):
        """ Crop image at position (predict_center_x,predict_center_y) and with size (WIDTH,HEIGHT) """
        return crop(image, self.predict_center_y - HEIGHT/2, self.predict_center_x - WIDTH/2, HEIGHT, WIDTH)  # top, left, height, width

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
                preds = self.predict(x,y)
                all_preds.append([x, y, preds])
                count += 1
        end = time.time()
        self.lbl_result_map.setText(f'{count} inferences in {end-start:.1f} s')
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
    args = parser.parse_args()

    rospy.init_node('explore')
    rate = rospy.Rate(0.5)
    app = QApplication(sys.argv)
    gui = ExploreWindow(args.calibration_folder)
    gui.show()
    sys.exit(app.exec_())
