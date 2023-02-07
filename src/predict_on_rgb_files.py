import os
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QFileDialog
from PIL import Image
import math
from raiv_libraries.rgb_cnn import RgbCnn
from raiv_libraries.cnn import Cnn

"""
Display a Qt window with images from a folder with their prediction from a model.
Highlight in red the wrong prediction (error of the model)
"""

SUCCESS_THRESHOLD = 50 # A success if prediction > threshold

class PredictOnImageFilesWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(PredictOnImageFilesWindow, self).__init__(parent)
        self.dir = QFileDialog.getExistingDirectory(self, "Select an image folder", "/common/work/images_storage", QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks)
        self.is_success_dir = self.dir.endswith('success')
        fname = QFileDialog.getOpenFileName(self, 'Open CKPT model file', '/common/work/model_trained', "Model files (*.ckpt)",
                                            options=QFileDialog.DontUseNativeDialog)
        self.wrong = self.total = 0
        self.model = RgbCnn.load_ckpt_model_file(fname[0])   # Load the selected model
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
            stat = f'Wrong predictions = {self.wrong} / {self.total}'
            self._lay.addWidget(QtWidgets.QLabel(stat))
            self._timer.stop()

    def compute_prediction(self, file):
        """ Compute the prediction [0,1] for a PIL cropped image """
        pil_rgb_img = Image.open(file).convert('RGB')
        pred = RgbCnn.predict_from_pil_rgb_image(self.model, pil_rgb_img)
        prob, cl = Cnn.compute_prob_and_class(pred)
        return prob

    def add_pixmap(self, pixmap, file_name, prob):
        if not pixmap.isNull():
            label_image = QtWidgets.QLabel(pixmap=pixmap)
            percentage = prob*100
            ch = f"{percentage:.2f}%, file = {file_name}"
            label_text = QtWidgets.QLabel(ch)
            self.total += 1
            if (percentage < SUCCESS_THRESHOLD and self.is_success_dir) or (percentage >= SUCCESS_THRESHOLD and not self.is_success_dir): # wrong prediction
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