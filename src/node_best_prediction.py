#!/usr/bin/env python3

import rospy
import random
import math
from raiv_research.srv import GetBestPrediction, GetBestPredictionResponse
from raiv_research.msg import Prediction, ListOfPredictions
from PIL import Image as PILImage
from sensor_msgs.msg import Image
import torch
from torchvision.transforms.functional import crop
from torchvision import transforms
from raiv_libraries.CNN import CNN
import cv2
import numpy as np


class NodeBestPrediction:
    """
    This node is both a service and a publisher.
    * Service best_prediction_service : use a GetBestPrediction message (no input message and a ListOfPredictions output message)
    When this service is called, return the current best prediction and invalidate all the predictions in its neighborhood.
    * Publisher : publish on the 'predictions' topic a ListOfPredictions message

    How to run?
    * roslaunch usb_cam usb_cam-test.launch   (to provide a /usb_cam/image_raw topic)
    * rosrun raiv_research node_visu_prediction.py   (to view the success/fail prediction points on the image)
    * rosrun raiv_research node_best_prediction.py   (to provide a /predictions and /new_image topics)
    * rosservice call /best_prediction_service  (to get the current best prediction. It loads a new image and invalidates the points in the picking zone)

    """
    def __init__(self):
        self.predictions = []
        msg_list_pred = ListOfPredictions()
        rospy.init_node('node_best_prediction')
        rospy.Service('best_prediction_service', GetBestPrediction, self._get_best_prediction)
        pub = rospy.Publisher('predictions', ListOfPredictions, queue_size=10)
        self.pub_image = rospy.Publisher('new_image', Image, queue_size=10)
        self._get_parameters()
        self._load_model()
        self.picking_point = None # No picking point yet
        self.image, self.width, self.height = self._get_image()  # Get the first image to process
        self.transform = transforms.Compose([
            transforms.Lambda(lambda img: self._crop_xy(img)),
            transforms.Resize(size=256),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        while not rospy.is_shutdown():
            x = random.randint(int(self.crop_width / 2), self.width - int(self.crop_width / 2)) # for a (x,y) centered crop to fit in the image
            y = random.randint(int(self.crop_height / 2), self.height - int(self.crop_height / 2))
            if self._ok_to_compute_proba(x, y): # Compute prediction only for necessary points (on an object, not in forbidden zone, ...)
                msg = Prediction()
                msg.x = x
                msg.y = y
                msg.proba = self._predict(x, y)
                self.predictions.append(msg)
                msg_list_pred.predictions = self.predictions
                pub.publish(msg_list_pred) # Publish the current list of predictions [ [x1,y1,prediction_1], ..... ]
            rospy.sleep(0.01)

    def _get_parameters(self):
        self.invalidation_radius = rospy.get_param('~invalidation_radius')  # When a prediction is selected, we invalidate all the previous predictions in this radius
        self.image_topic = rospy.get_param('~image_topic')
        self.crop_width = rospy.get_param('~crop_width') # Size of cropped image
        self.crop_height = rospy.get_param('~crop_height')
        self.model_name = rospy.get_param('~model_name')
        # min and max HSV values for color thresholding for object recognition (max S and V = 255, max H = 180)
        self.low_h = rospy.get_param('~low_h')
        self.high_h = rospy.get_param('~high_h')
        self.low_s = rospy.get_param('~low_s')
        self.high_s = rospy.get_param('~high_s')
        self.low_v = rospy.get_param('~low_v')
        self.high_v = rospy.get_param('~high_v')
        # Position of the object box in pixel
        self.px_min = rospy.get_param('~px_min')
        self.px_max = rospy.get_param('~px_max')
        self.py_min = rospy.get_param('~py_min')
        self.py_max = rospy.get_param('~py_max')

    def _ok_to_compute_proba(self, x, y):
        """ Return True if this (x,y) point is a good candidate i.e. is on an object and not in the picking zone.
        Prediction will be computed only for good candidate points.
        """
        # Test if inside the picking zone
        if self.picking_point and math.dist((self.picking_point[0], self.picking_point[1]), (x, y)) < self.invalidation_radius:
            return False  # This point is inside the picking zone
        # Test if on an object
        if self.object_image[y, x] == 0:  # On a black
            return False  # Not on an object (it is on a black pixel)
        # test if it's on the box zone
        if x < self.px_min or x > self.px_max or y < self.py_min or y > self.py_max:
            return False
        return True  # If all tests succeed, it's a good candidate.

    def _compute_object_image(self,pil_image):
        """
        From the RGB pil_image, compute a binary image where white pixels = object and black ones = background.
        """
        opencv_rgb_image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
        frame_HSV = cv2.cvtColor(opencv_rgb_image, cv2.COLOR_BGR2HSV)
        self.object_image = cv2.inRange(frame_HSV, (self.low_h, self.low_s, self.low_v), (self.high_h, self.high_s, self.high_v))  # Binarize the colored image
        cv2.imwrite("test.png", self.object_image)

    def _load_model(self):
        """
        Load a pretrained 'resnet18' model from a CKPT filename, freezed for inference
        """
        self.model = CNN(backbone='resnet18')
        self.inference_model = self.model.load_from_checkpoint(self.model_name)   #  Load the selected model
        self.inference_model.freeze()

    def _predict(self, x, y):
        """ Predict probability and class for a cropped image centered at (x,y) """
        self.predict_center_x = x
        self.predict_center_y = y
        img = self.transform(self.image)  # Get the cropped transformed image (size = [CROP_WIDTH,CROP_HEIGHT])
        img = img.unsqueeze(0)  # To have a 4-dim tensor ([nb_of_images, channels, w, h])
        _, preds = self._evaluate_image(img, self.inference_model)
        pred = torch.exp(preds)
        return pred[0][1].item()  # Return the success probability

    @torch.no_grad()
    def _evaluate_image(self, image, model):
        features, prediction = model(image)
        return features.detach().numpy(), prediction.detach()

    def _crop_xy(self, image):
        """ Crop image at position (predict_center_x,predict_center_y) and with size (WIDTH,HEIGHT) """
        return crop(image, self.predict_center_y - self.crop_height / 2, self.predict_center_x - self.crop_width / 2, self.crop_height, self.crop_width)  # top, left, height, width

    def _get_image(self):
        """
        Recover an image from the IMAGE_TOPIC topic
        :return: an RGB image
        """
        msg_image = rospy.wait_for_message(self.image_topic, Image)
        self.pub_image.publish(msg_image)
        pil_image = self._to_pil(msg_image)
        self._compute_object_image(pil_image)
        return pil_image, msg_image.width, msg_image.height

    def _to_pil(self, msg):
        """ Recover the image in the msg sensor_msgs.Image message and convert it to a PILImage"""
        size = (msg.width, msg.height)  # Image size
        img = PILImage.frombytes('RGB', size, msg.data)  # sensor_msg Image to PILImage
        return img

    def _get_best_prediction(self, req):
        """ best_prediction_service service callback which return a Prediction message (the best, highest one)"""
        # Get a new image to process
        self.image, width, height = self._get_image()
        # Find best prediction
        best_prediction = self.predictions[0]
        for prediction in self.predictions:
            if prediction.proba > best_prediction.proba:
                best_prediction = prediction
        self._invalidate_neighborhood(best_prediction.x, best_prediction.y)
        self.picking_point = (best_prediction.x, best_prediction.y)
        return GetBestPredictionResponse(best_prediction)

    def _invalidate_neighborhood(self, x, y):
        """ Invalidate (remove) all the predictions in a circle of radius INVALIDATION_RADIUS centered in (x,y)"""
        self.predictions = [pred for pred in self.predictions if math.dist((pred.x, pred.y), (x, y)) > self.invalidation_radius]


if __name__ == '__main__':
    try:
        n = NodeBestPrediction()
    except rospy.ROSInterruptException:
        pass