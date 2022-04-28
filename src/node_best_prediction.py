#!/usr/bin/env python3

import rospy
import random
import math
from raiv_research.srv import GetBestPrediction, GetBestPredictionResponse
from raiv_research.msg import Prediction, ListOfPredictions
from raiv_libraries.srv import get_coordservice
from raiv_libraries.get_coord_node import InBoxCoord
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
        # Provide the 'best_prediction_service' service
        rospy.Service('best_prediction_service', GetBestPrediction, self._get_best_prediction)
        # Publish the 'predictions' topic (a list of all Prediction)
        pub = rospy.Publisher('predictions', ListOfPredictions, queue_size=10)
        self._get_parameters()
        self._load_model()
        self.picking_point = None # No picking point yet
        self.transform = transforms.Compose([
            transforms.Lambda(lambda img: self._crop_xy(img)),
            transforms.Resize(size=256),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        rospy.wait_for_service('In_box_coordService')
        coord_serv = rospy.ServiceProxy('In_box_coordService', get_coordservice)
        while not rospy.is_shutdown():
            # Ask 'In_box_coordService' service for a random point in the picking box located on one of the objects
            resp = coord_serv('random', InBoxCoord.PICK, InBoxCoord.ON_OBJECT, self.crop_width, self.crop_height, None, None)
            # Compute prediction only for necessary points (on an object, not in forbidden zone, ...)
            if self._not_in_picking_zone(resp.x_pixel, resp.y_pixel):
                msg = Prediction()
                msg.x = resp.x_pixel
                msg.y = resp.y_pixel
                msg.proba = self._predict(resp.x_pixel, resp.y_pixel, resp.rgb_crop)
                self.predictions.append(msg)
                msg_list_pred.predictions = self.predictions
                pub.publish(msg_list_pred)  # Publish the current list of predictions [ [x1,y1,prediction_1], ..... ]
            rospy.sleep(0.01)


    def _get_parameters(self):
        # self.invalidation_radius = rospy.get_param('~invalidation_radius')  # When a prediction is selected, we invalidate all the previous predictions in this radius
        # self.image_topic = rospy.get_param('~image_topic')
        # self.crop_width = rospy.get_param('~crop_width') # Size of cropped image
        # self.crop_height = rospy.get_param('~crop_height')
        # self.model_name = rospy.get_param('~model_name')
        self.invalidation_radius = 150  # When a prediction is selected, we invalidate all the previous predictions in this radius
        self.image_topic = "/RGBClean"
        self.crop_width = 50 # Size of cropped image
        self.crop_height = 50
        self.model_name = "/common/modele/model-epoch=16-val_loss=0.07.ckpt"


    def _not_in_picking_zone(self, x, y):
        """ Return True if this (x,y) point is a good candidate i.e. is not in the invalidated zone (current picking zone).
        Prediction will be computed only for good candidate points.
        """
        # Test if inside the picking zone (so, not a good candidate)
        return not (self.picking_point and math.dist((self.picking_point[0], self.picking_point[1]), (x, y)) < self.invalidation_radius)


    def _load_model(self):
        """
        Load a pretrained 'resnet18' model from a CKPT filename, freezed for inference
        """
        self.model = CNN(backbone='resnet18')
        self.inference_model = self.model.load_from_checkpoint(self.model_name)   #  Load the selected model
        self.inference_model.freeze()


    def _predict(self, x, y, rgb_crop):
        """ Predict probability and class for a cropped image centered at (x,y) """
        self.predict_center_x = x
        self.predict_center_y = y
        img = self.transform(rgb_crop)  # Get the cropped transformed image (size = [CROP_WIDTH,CROP_HEIGHT])
        img = img.unsqueeze(0)  # To have a 4-dim tensor ([nb_of_images, channels, w, h])
        _, preds = self._evaluate_image(img, self.inference_model)
        pred = torch.exp(preds)
        return pred[0][1].item()  # Return the success probability


    @torch.no_grad()
    def _evaluate_image(self, image, model):
        features, prediction = model(image)
        return features.detach().numpy(), prediction.detach()


    def _crop_xy(self, msg_image):
        """ Crop image at position (predict_center_x,predict_center_y) and with size (WIDTH,HEIGHT) """
        pil_image = self._to_pil(msg_image)
        return crop(pil_image, self.predict_center_y - self.crop_height / 2, self.predict_center_x - self.crop_width / 2, self.crop_height, self.crop_width)  # top, left, height, width


    def _to_pil(self, msg):
        """ Recover the image in the msg sensor_msgs.Image message and convert it to a PILImage"""
        size = (msg.width, msg.height)  # Image size
        img = PILImage.frombytes('RGB', size, msg.data)  # sensor_msg Image to PILImage
        imgArray = np.array(img)
        return img


    def _get_best_prediction(self, req):
        """ best_prediction_service service callback which return a Prediction message (the best, highest one)"""
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