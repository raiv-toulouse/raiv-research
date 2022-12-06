#!/usr/bin/env python3

import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import Qt, QPoint
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
        self.sb_low.valueChanged.connect(self._low_value_change)
        self.sb_high.valueChanged.connect(self._high_value_change)
        self.prediction_min_threshold = self.sb_low.value() / 100 # Percent => 0.xx
        self.prediction_max_threshold = self.sb_high.value() / 100
        self.predictions = None
        self.image = None

    def _low_value_change(self):
        self.prediction_min_threshold = min(self.sb_low.value() / 100, self.prediction_max_threshold)
        self.update()

    def _high_value_change(self):
        self.prediction_max_threshold = max(self.sb_high.value() / 100, self.prediction_min_threshold)
        self.update()

    def _change_image(self, req):
        """ When a new webcam image arrives, store it in self.image """
        format = QImage.Format_RGB888
        image = QImage(req.data, req.width, req.height, format)
        self.image = image

    def _update_predictions(self,data):
        """ When a new list of predictions arrives, draw them """
        self.predictions = data.predictions
        self.update()

    def _draw_histogram(self):
        # generate the plot
        self.ax = self.gv_plot.canvas.ax
        self.ax.cla()
        # and the list of prediction's values
        preds = [100*p.proba for p in self.predictions]
        print(preds)
        # Plot a histogram in 100 bins
        N, bins, patches = self.ax.hist(preds, bins=100, range=(0,100), edgecolor='black', linewidth=1)
        # The color of the histogram's bars depends on proba value
        lower_percent = self.prediction_min_threshold * 100
        higher_percent = self.prediction_max_threshold * 100
        for i in range(len(N)):
            if i < lower_percent:
                patches[i].set_facecolor('red')
            elif i > higher_percent:
                patches[i].set_facecolor("green")
            else:
                patches[i].set_facecolor("blue")
        self.gv_plot.canvas.draw_idle()

    def paintEvent(self, event):
        """ Display the last webcam image and draw predictions (green points if prediction > THRESHOLD otherwise red) """
        qp = QPainter(self)  #.lbl_image)
        rect = event.rect()
        point_size = 3
        if self.image:
            qp.drawImage(rect, self.image, rect)
        if self.predictions:
            for prediction in self.predictions:
                x = prediction.x
                y = prediction.y
                if prediction.proba > self.prediction_max_threshold:
                    qp.setPen(QPen(Qt.green, point_size))
                elif prediction.proba < self.prediction_min_threshold:
                    qp.setPen(QPen(Qt.red, point_size))
                else: # Between min and max threshold
                    qp.setPen(QPen(Qt.blue, point_size))
                qp.drawPoint(x, y)
            # First one is the best proba (so the futur picking point)
            best_pred = self.predictions[0]
            qp.setPen(QPen(Qt.magenta, 2*point_size))
            qp.drawPoint(best_pred.x, best_pred.y)
            self.lbl_best_pred.setText(f'{best_pred.proba*100:.2f}')
            # Draw the histogram
            self._draw_histogram()
        qp.end()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = NodeVisuPrediction()
    gui.show()
    sys.exit(app.exec_())