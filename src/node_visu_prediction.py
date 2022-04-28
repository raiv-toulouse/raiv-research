#!/usr/bin/env python3

import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import Qt
from PyQt5.uic import loadUi
import rospy
from raiv_research.msg import ListOfPredictions
from sensor_msgs.msg import Image


class NodeVisuPrediction(QWidget):
    """
    Display predictions on an image.
    Subscribe to predictions topic to get all the predictions (from best_prediction node)
    Subscribe to new_image topic to get the new current image and replace the previous one.
    """
    def __init__(self):
        super().__init__()
        loadUi("node_visu_prediction.ui", self)
        rospy.init_node('node_visu_prediction')
        rospy.Subscriber("predictions", ListOfPredictions, self._update_predictions)
        rospy.Subscriber('new_image', Image, self._change_image)
        self.prediction_threshold = rospy.get_param('~prediction_threshold') # threshold for success grasping prediction
        self.predictions = None
        self.image = None

    def _change_image(self, req):
        print('coucou')
        """ When a new webcam image arrives, store it in self.image """
        format = QImage.Format_RGB888
        image = QImage(req.data, req.width, req.height, format)
        self.image = image

    def _update_predictions(self,data):
        """ When a new list of predictions arrives, draw them """
        self.predictions = data.predictions
        self.repaint()

    def paintEvent(self, event):
        """ Display the last webcam image and draw predictions (green points if prediction > THRESHOLD otherwise red) """
        qp = QPainter(self)
        rect = event.rect()
        if self.image:
            qp.drawImage(rect, self.image, rect)
        if self.predictions:
            for prediction in self.predictions:
                x = prediction.x
                y = prediction.y
                if prediction.proba > self.prediction_threshold:
                    qp.setPen(QPen(Qt.green, 3))
                else:
                    qp.setPen(QPen(Qt.red, 3))
                qp.drawPoint(x, y)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = NodeVisuPrediction()
    gui.show()
    sys.exit(app.exec_())