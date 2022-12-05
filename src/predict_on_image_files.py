import os
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QPen
from PyQt5.QtWidgets import QFileDialog
from raiv_libraries.prediction_tools import PredictTools
from raiv_libraries.image_tools import ImageTools
from PIL import Image
import cv2
import torch
import math

"""
Display a Qt window with images from a folder with their prediction from a model.
Highlight in red the wrong prediction (error of the model)
"""

SUCCESS_THRESHOLD = 50 # A success if prediction > threshold

class PredictOnImageFilesWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(PredictOnImageFilesWindow, self).__init__(parent)
        self.dir = QFileDialog.getExistingDirectory(self, "Select an image folder", "/common/work/stockage_banque_image", QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks)
        self.is_success_dir = self.dir.endswith('success')
        fname = QFileDialog.getOpenFileName(self, 'Open CKPT model file', '/common/work/model_trained', "Model files (*.ckpt)",
                                            options=QFileDialog.DontUseNativeDialog)
        self.wrong = self.total = 0
        self.image_model, self.inference_model = PredictTools.load_model(fname[0])  # Load the selected models
        self.scrollArea = QtWidgets.QScrollArea(widgetResizable=True)
        self.setCentralWidget(self.scrollArea)
        content_widget = QtWidgets.QWidget()
        self.scrollArea.setWidget(content_widget)
        self._lay = QtWidgets.QVBoxLayout(content_widget)
        self.file_names_it = iter([file for file in os.listdir(self.dir)])
        self._timer = QtCore.QTimer(self, interval=1)
        self._timer.timeout.connect(self.on_timeout)
        self._timer.start()

    def on_timeout(self):
        try:
            file_name = next(self.file_names_it)
            file = os.path.join(self.dir, file_name)
            prob = self.compute_prediction(file)
            pixmap = QtGui.QPixmap(file)
            self.add_pixmap(pixmap, file_name, prob)
        except StopIteration:
            print('fin')
            stat = f'Wrong predictions = {self.wrong} / {self.total}'
            self._lay.addWidget(QtWidgets.QLabel(stat))
            self._timer.stop()

    def compute_prediction(self, file):
        image = Image.open(file).convert('RGB')
        img = ImageTools.transform_image(image)  # Get the loaded images, resize in 224 and transformed in tensor
        img = img.unsqueeze(0)  # To have a 4-dim tensor ([nb_of_images, channels, w, h])
        pred = PredictTools.predict(self.image_model, img)
        prob, cl = self._compute_prob_and_class(pred)
        return math.floor(prob)

    def _compute_prob_and_class(self, pred):
        """ Retrieve class (success or fail) and its associated percentage from pred """
        prob, cl = torch.max(pred, 1)
        if cl.item() == 0:  # Fail
            prob = 100 * (1 - prob.item())
        else:  # Success
            prob = 100 * prob.item()
        return prob, cl

    def add_pixmap(self, pixmap, file_name, prob):
        if not pixmap.isNull():
            label_image = QtWidgets.QLabel(pixmap=pixmap)
            ch = f"{prob}%, file = {file_name}"
            label_text = QtWidgets.QLabel(ch)
            self.total += 1
            if (prob < 50 and self.is_success_dir) or (prob >= 50 and not self.is_success_dir): # wrong prediction
                label_text.setStyleSheet("QLabel { background-color : red; }")
                self.wrong += 1
            hBox = QtWidgets.QHBoxLayout()
            hBox.addWidget(label_image)
            hBox.addWidget(label_text)
            self._lay.addLayout(hBox)

if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    w = PredictOnImageFilesWindow()
    w.show()
    sys.exit(app.exec_())