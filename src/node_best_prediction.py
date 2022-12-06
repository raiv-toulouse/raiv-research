#!/usr/bin/env python3

import rospy
import math
from raiv_research.srv import GetBestPrediction, GetBestPredictionResponse
from raiv_research.msg import Prediction, ListOfPredictions
from raiv_libraries.srv import get_coordservice
from raiv_libraries.srv import PickingBoxIsEmpty
from raiv_libraries.srv import ClearPrediction, ClearPredictionResponse
from raiv_research.srv import ProcessNewImage, ProcessNewImageResponse
from raiv_libraries.get_coord_node import InBoxCoord
import torch
from raiv_libraries.image_tools import ImageTools
from raiv_libraries.prediction_tools import PredictTools
from sensor_msgs.msg import Image


IMAGE_TOPIC = "/camera/color/image_raw"


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
        # Provide the 'best_prediction_service' service
        rospy.Service('best_prediction_service', GetBestPrediction, self._process_service)
        self.pub_image = rospy.Publisher('new_image', Image, queue_size=10)
        self._process_new_image(None)
        # Publish the 'predictions' topic (a list of all Prediction)
        self.pub_predictions = rospy.Publisher('predictions', ListOfPredictions, queue_size=10)
        self.invalidation_radius = invalidation_radius  # When a prediction is selected, we invalidate all the previous predictions in this radius
        self.image_topic = image_topic
        self.model_path = ckpt_model_file
        self.model, self.inference_model = PredictTools.load_model(self.model_path)
        self.picking_point = None # No picking point yet
        self.prediction_processing = False
        ### Appel du service emptybox
        rospy.wait_for_service('/Is_Picking_Box_Empty')
        self.is_picking_box_empty_service = rospy.ServiceProxy('/Is_Picking_Box_Empty', PickingBoxIsEmpty)
        rospy.Service('Clear_Prediction', ClearPrediction, self._clear_service)
        rospy.Service('Process_new_image', ProcessNewImage, self._process_new_image)
        rospy.wait_for_service('In_box_coordService')
        self.coord_serv = rospy.ServiceProxy('In_box_coordService', get_coordservice)

    def process_bin_picking(self):
        # Message used to send the list of all predictions
        msg_list_pred = ListOfPredictions()
        #self.coord_serv('random', InBoxCoord.PICK, InBoxCoord.ON_OBJECT, ImageTools.CROP_WIDTH, ImageTools.CROP_HEIGHT, None, None)
        while not self._is_picking_box_empty():
            if self.prediction_processing:
                # Ask 'In_box_coordService' service for a random point in the picking box located on one of the objects
                resp = self.coord_serv('random_no_refresh', InBoxCoord.PICK, InBoxCoord.ON_OBJECT, ImageTools.CROP_WIDTH, ImageTools.CROP_HEIGHT, None, None)
                #if self._not_in_picking_zone(resp.x_pixel, resp.y_pixel):   # Compute prediction only for necessary points (on an object, not in forbidden zone, ...)
                msg = Prediction()
                msg.x = resp.x_pixel
                msg.y = resp.y_pixel
                image_pil = ImageTools.ros_msg_to_pil(resp.rgb_crop)
                # Compute the prediction for this cropped image
                pred = PredictTools.predict_from_pil_rgb_image(self.model, image_pil)
                msg.proba = PredictTools.compute_prob_and_class(pred)
                self.predictions.append(msg)
                self.predictions.sort(key=lambda x: x.proba, reverse=True)  # sort by decreasing proba
                msg_list_pred.predictions = self.predictions
                self.pub_predictions.publish(msg_list_pred)  # Publish the current list of predictions [ [x1,y1,prediction_1], ..... ]
            rospy.sleep(0.01)
        print('End of bin picking operation')


    def _is_picking_box_empty(self):
        """
        Test if picking box is empty
        """
        picking_box_empty = self.is_picking_box_empty_service().empty_box
        if picking_box_empty:
            self.predictions = []
        return picking_box_empty

    def _process_new_image(self, req):
        """
        Get a new image and publish it to the new_image topic (for node_visu_prediction.py )
        """
        msg_image = rospy.wait_for_message(IMAGE_TOPIC, Image)
        self.pub_image.publish(msg_image)
        self.prediction_processing = True
        return ProcessNewImageResponse()

    def _process_service(self, req):
        # Find best prediction
        if not self.predictions: # This list is empty
            raise rospy.ServiceException("self.predictions : empty list")
        else:  # Return the best prediction from 'predictions' list
            best_prediction = self.predictions[0]
            for prediction in self.predictions:
                if prediction.proba > best_prediction.proba:
                    best_prediction = prediction
        self.picking_point = (best_prediction.x, best_prediction.y)
        return GetBestPredictionResponse(best_prediction)

    def _clear_service(self, req):
        self.predictions = []
        self.prediction_processing = False
        return ClearPredictionResponse()

    #
    # If we take into account an invalidation radius
    #
    def _invalidate_neighborhood(self, x, y):
        """ Invalidate (remove) all the predictions in a circle of radius INVALIDATION_RADIUS centered in (x,y)"""
        self.predictions = [pred for pred in self.predictions if math.dist((pred.x, pred.y), (x, y)) > self.invalidation_radius]

    def _not_in_picking_zone(self, x, y):
        """ Return True if this (x,y) point is a good candidate i.e. is not in the invalidated zone (current picking zone).
        Prediction will be computed only for good candidate points.
        """
        # Test if inside the picking zone (so, not a good candidate)
        return not (self.picking_point and math.dist((self.picking_point[0], self.picking_point[1]), (x, y)) < self.invalidation_radius)



if __name__ == '__main__':
    import argparse
    # Analyse arguments
    parser = argparse.ArgumentParser(description='Compute a list of predictions for random points and provide the best one as a service.')
    parser.add_argument('ckpt_model_file', type=str, help='CKPT model file')
    parser.add_argument('--image_topic', type=str, default="/camera/color/image_raw", help='Topic which provides an image')
    parser.add_argument('--invalidation_radius', type=int, default=30, help='Radius in pixels where predictions will be invalidated')
    args = parser.parse_args()
    try:
        node_best_pred = NodeBestPrediction(args.ckpt_model_file, args.invalidation_radius, args.image_topic)
        node_best_pred.process_bin_picking()
    except rospy.ROSInterruptException:
        pass