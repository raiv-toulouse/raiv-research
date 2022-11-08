#!/usr/bin/env python3

import rospy
import math
from raiv_research.srv import GetBestPrediction, GetBestPredictionResponse
from raiv_research.msg import Prediction, ListOfPredictions
from raiv_libraries.srv import get_coordservice
from raiv_libraries.srv import PickingBoxIsEmpty, PickingBoxIsEmptyResponse
from raiv_libraries.srv import ClearPrediction, ClearPredictionResponse
from raiv_libraries.get_coord_node import InBoxCoord
from PIL import Image as PILImage
import numpy as np
import torch
from raiv_libraries.CNN import CNN
from raiv_libraries.image_tools import ImageTools
from raiv_libraries.prediction_tools import PredictTools
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time


IMAGE_TOPIC = "/camera/color/image_raw"
THRESHOLD_ABOVE_TABLE = 10

class NodeBestPrediction:
    """
    This node is both a service and a publisher.
    * Service best_prediction_service : use a GetBestPrediction message (no input message and a ListOfPredictions output message)
    When this service is called, return the current best prediction and invalidate all the predictions in its neighborhood.
    * Publisher : publish on the 'predictions' topic a ListOfPredictions message

    How to run?
    * roslaunch realsense2_camera rs_camera.launch align_depth:=true (to provide a /camera/color/image_raw topic)
    * rosrun raiv_research node_visu_prediction.py   (to view the success/fail prediction points on the image)
    * rosrun raiv_research node_best_prediction.py CKPT_FILE --invalidation_radius INT --image_topic STR (to provide a /predictions topic)
    * rosservice call /best_prediction_service  (to get the current best prediction. It loads a new image and invalidates the points in the picking zone)

    """
    def __init__(self, ckpt_model_file, invalidation_radius, image_topic):
        rospy.init_node('node_best_prediction')
        # List of all predictions made for some random points
        self.predictions = []
        # Message used to send the list of all predictions
        msg_list_pred = ListOfPredictions()
        # Provide the 'best_prediction_service' service
        rospy.Service('best_prediction_service', GetBestPrediction, self.process_service)
        self.pub_image = rospy.Publisher('new_image', Image, queue_size=10)
        self._get_new_image()
        # Publish the 'predictions' topic (a list of all Prediction)
        pub = rospy.Publisher('predictions', ListOfPredictions, queue_size=10)
        self.invalidation_radius = invalidation_radius  # When a prediction is selected, we invalidate all the previous predictions in this radius
        self.image_topic = image_topic
        self.model_path = ckpt_model_file
        self.model, self.inference_model = PredictTools.load_model(self.model_path)
        self.picking_point = None # No picking point yet
        compt = 1
        ### Appel du service emptybox
        rospy.wait_for_service('/Empty_Picking_Box')
        self.call_service = rospy.ServiceProxy('/Empty_Picking_Box', PickingBoxIsEmpty)
        rospy.Service('Clear_Prediction', ClearPrediction, self.clear_service)
        rospy.wait_for_service('In_box_coordService')
        coord_serv = rospy.ServiceProxy('In_box_coordService', get_coordservice)
        resp = coord_serv('random', InBoxCoord.PICK, InBoxCoord.ON_OBJECT, ImageTools.CROP_WIDTH, ImageTools.CROP_HEIGHT, None, None)
        while not self._continue_to_picking():
            # Ask 'In_box_coordService' service for a random point in the picking box located on one of the objects
            resp = coord_serv('random_no_refresh', InBoxCoord.PICK, InBoxCoord.ON_OBJECT, ImageTools.CROP_WIDTH, ImageTools.CROP_HEIGHT, None, None)
            # Compute prediction only for necessary points (on an object, not in forbidden zone, ...)
            #if self._not_in_picking_zone(resp.x_pixel, resp.y_pixel):
            msg = Prediction()
            msg.x = resp.x_pixel
            msg.y = resp.y_pixel
            image_pil = ImageTools.sensor_msg_to_pil(resp.rgb_crop)

            """this couple lines serve to save the image before prediction"""
            # save_image = PILImage.fromarray(save_image_bgr)
            #compt = compt + 1
            #save_image.save("/common/work/stockage_image_test/test"+str(compt)+".png")

            """transformation de l'image rgb en bgr"""
            image_rgb = ImageTools.pil_to_numpy(image_pil)
            image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2RGB)
            image_bgr_pil = ImageTools.numpy_to_pil(image_bgr)

            img = ImageTools.transform_image(image_pil)  # Get the cropped transformed RGB image
            #img = ImageTools.transform_image(image_bgr_pil)  # Get the cropped transformed BGR image
            img = img.unsqueeze(0)  # To have a 4-dim tensor ([nb_of_images, channels, w, h])
            pred = PredictTools.predict(self.model, img)
            msg.proba = PredictTools.compute_prob_and_class(pred)
            self.predictions.append(msg)
            msg_list_pred.predictions = self.predictions
            pub.publish(msg_list_pred)  # Publish the current list of predictions [ [x1,y1,prediction_1], ..... ]
            rospy.sleep(0.01)
        print('dévracage fini')


    def _continue_to_picking(self):
        picking_box_empty = self.call_service().empty_box
        if picking_box_empty==True:
            self.predictions = []
        return picking_box_empty


    def _not_in_picking_zone(self, x, y):
        """ Return True if this (x,y) point is a good candidate i.e. is not in the invalidated zone (current picking zone).
        Prediction will be computed only for good candidate points.
        """
        # Test if inside the picking zone (so, not a good candidate)
        return not (self.picking_point and math.dist((self.picking_point[0], self.picking_point[1]), (x, y)) < self.invalidation_radius)

    def _get_new_image(self):
        # Get a new image and publish it to the new_image topic(for node_visu_prediction.py )
        msg_image = rospy.wait_for_message(IMAGE_TOPIC, Image)
        self.pub_image.publish(msg_image)

    @torch.no_grad()
    def _evaluate_image(self, image, model):
        features, prediction = model(image)
        return features.detach().numpy(), prediction.detach()


    def process_service(self, req):
        self._get_new_image()

        if req.mode == 'classic':
            precis = req.precision
            prediction = self._get_best_prediction(high=False, manual=False)
        elif req.mode == 'high_mode':
            precis = req.precision
            prediction = self._get_best_prediction(classic=False, manual=False)
        elif req.mode == 'manual':
            precis = req.precision
            prediction = self._get_best_prediction(precis, classic=False, high=False)

        return GetBestPredictionResponse(prediction)


    def _get_best_prediction(self, classic=True, high=True, manual=True, precis=None):
        # Find best prediction
        if not(self.predictions): # This list is empty
            raise rospy.ServiceException("self.predictions : empty list")
        if classic==True:
            self.best_prediction = self.predictions[0]
            for prediction in self.predictions:
                if prediction.proba > self.best_prediction.proba:
                    self.best_prediction = prediction

        if high==True:
            proba = 0.5
            while proba < 0.7:
                self.best_prediction = self.predictions[0]
                for prediction in self.predictions:
                    if prediction.proba > self.best_prediction.proba:
                        self.best_prediction = prediction
                        proba = float(self.best_prediction.proba)

        if manual==True:
            proba = 0
            seuil = float(precis)
            while proba < seuil:
                self.best_prediction = self.predictions[0]
                for prediction in self.predictions:
                    if prediction.proba > self.best_prediction.proba:
                        self.best_prediction = prediction
                        proba = float(self.best_prediction.proba)





        self.picking_point = (self.best_prediction.x, self.best_prediction.y)
        return self.best_prediction


    def _invalidate_neighborhood(self, x, y):
        """ Invalidate (remove) all the predictions in a circle of radius INVALIDATION_RADIUS centered in (x,y)"""
        self.predictions = [pred for pred in self.predictions if math.dist((pred.x, pred.y), (x, y)) > self.invalidation_radius]

    def clear_service(self, req):
        self._get_new_image()
        self.predictions = []
        if len(self.predictions)==0:
            flag = True
        else:
            flag = False
        return ClearPredictionResponse(clear=flag)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Compute a list of predictions for random points and provide the best one as a service.')
    parser.add_argument('ckpt_model_file', type=str, help='CKPT model file')
    parser.add_argument('--image_topic', type=str, default="/camera/color/image_raw", help='Topic which provides an image')
    parser.add_argument('--invalidation_radius', type=int, default=300, help='Radius in pixels where predictions will be invalidated')
    args = parser.parse_args()

    try:
        n = NodeBestPrediction(args.ckpt_model_file, args.invalidation_radius, args.image_topic)
    except rospy.ROSInterruptException:
        pass