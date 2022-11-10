import sys
from PyQt5.QtWidgets import *
from PyQt5 import uic
import cv2
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
from raiv_libraries.image_tools import ImageTools
from PIL import Image
from raiv_libraries.prediction_tools import PredictTools

# global variables
Z_PICK_ROBOT = 0.15  # Z coord before going down to pick
X_OUT = 0.0  # XYZ coord where the robot is out of camera scope
Y_OUT = -0.3
Z_OUT = 0.12

### Used for DEBUG purpose
matplotlib.use('Qt5Agg')


def imshow(images, title=None, pil_image=False):
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
        uic.loadUi("explore_ihm.ui", self)  # needs the canvas_explore.py file in the current directory
        self.title = 'Camera'
        # event handlers
        self.btn_load_model.clicked.connect(self._load_model)
        self.btn_load_images.clicked.connect(self.predict_from_image)
        self.btn_change_image.clicked.connect(self.move_robot)
        self.btn_activate_robot.clicked.connect(self._activate_robot)
        self.sb_threshold.valueChanged.connect(self._change_threshold)
        # attributs
        self.dPoint = PerspectiveCalibration(calibration_folder)
        self.image_controller = SimpleImageController(image_topic='/camera/color/image_raw')
        self.image_model = None
        self.inference_model = None
        self.robot = None
        self._set_image()
        self._load_model()

    def predict_from_point(self, x, y):
        """ Predict probability and class for a cropped image at (x,y) """
        self.predict_center_x = x
        self.predict_center_y = y
        bgr_crop_pil = ImageTools.crop_xy(self.image, x, y)
        bgr_crop = ImageTools.pil_to_numpy(bgr_crop_pil)
        rgb_crop = cv2.cvtColor(bgr_crop, cv2.COLOR_BGR2RGB)
        rgb_crop_pil = ImageTools.numpy_to_pil(rgb_crop)
        img = ImageTools.transform_image(rgb_crop_pil)  # Get the cropped 224 transformed image for rgb model
        #img = ImageTools.transform_image(bgr_crop_pil)  # Get the cropped 224 transformed image for bgr model
        img = img.unsqueeze(0)  # To have a 4-dim tensor ([nb_of_images, channels, w, h])
        return self.predict(img)

    def predict_from_image(self):
        """ Load the images data """
        loaded_image = QFileDialog.getOpenFileName(self, 'Open image', '.', "Model files (*.png)",
                                                   options=QFileDialog.DontUseNativeDialog)
        if loaded_image[0]:
            self.image = Image.open(loaded_image[0])
        image_bgr = ImageTools.pil_to_numpy(self.image)
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        self.image = ImageTools.numpy_to_pil(image_rgb)
        img = ImageTools.transform_image(self.image)  # Get the loaded images, resize in 224 and transformed in tensor
        img = img.unsqueeze(0)  # To have a 4-dim tensor ([nb_of_images, channels, w, h])
        pred = self.predict(img)
        prob, cl = self.canvas._compute_prob_and_class(pred)
        self.prediction_from_image.setText("La prédiction de l'image chargé est : " + str(prob) + " %")
        print(prob)

    def predict(self, img):
        pred = PredictTools.predict(self.image_model, img)
        return pred

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
            ckpt_model_name = os.path.basename(fname[0])  # Only the name, without path
            ckpt_model_dir = os.path.dirname(fname[0])
            ckpt_model_path = str(ckpt_model_dir + '/' + ckpt_model_name)
            self.image_model, self.inference_model = PredictTools.load_model(ckpt_model_path)  # Load the selected models
            self.lbl_model_name.setText(ckpt_model_name)

    def _activate_robot(self):
        """ """
        self.robot = Robot_with_vaccum_gripper()
        self.btn_change_image.setEnabled(True)

    def move_robot(self):
        """  Move robot out of camera scope then get and display a new image """
        self.robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT)
        self._set_image()

    # @torch.no_grad()
    # def _evaluate_image(self, image):
    #     features, prediction = self.inference_model(image)
    #     return features.detach().numpy(), prediction.detach()

    def _set_image(self):
        """ Get an image from topic and display it on the canvas """
        img, width, height = self.image_controller.get_image()
        self.canvas.set_image(img)
        self.image = img

    def _compute_all_preds(self, start_coord, end_coord):
        """ Compute a list of predictions like :
        [ [x, y, tensor([[prob_fail, proba_success]])], ...] with x,y the center of cropped image size (WIDTH,HEIGHT)
        """
        start = time.time()
        fichier = open('/common/predictions_explore.txt', 'w')
        txt = 'Prédictions de explore.py'
        fichier.write(txt.center(20,'*'))
        all_preds = []
        steps = int(self.edt_nb_pixels_per_step.text())
        count = 0
        for x in range(start_coord.x(), end_coord.x(), steps):
            for y in range(start_coord.y(), end_coord.y(), steps):
                preds = self.predict_from_point(x, y)
                all_preds.append([x, y, preds])
                texte = '\n' + str(x) + ', ' + str(y) + ', ' + str(preds[0][1].item())
                fichier.write(texte)
                count += 1
        end = time.time()
        testimg = self.image
        imgArray = np.asarray(testimg)
        img_rgb = cv2.cvtColor(imgArray, cv2.COLOR_RGB2BGR)
        cv2.imwrite('/common/image_predictions.jpg', img_rgb)
        fichier.close()
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
    args = parser.parse_args()

    rospy.init_node('explore')
    rate = rospy.Rate(0.5)
    app = QApplication(sys.argv)
    gui = ExploreWindow(args.calibration_folder)
    gui.show()
    sys.exit(app.exec_())
