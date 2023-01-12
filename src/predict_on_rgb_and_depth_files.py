import os
import pathlib

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QFileDialog, QMessageBox
from PIL import Image
import math
from raiv_libraries.cnn import Cnn
from raiv_libraries.rgb_and_depth_cnn import RgbAndDepthCnn

"""
Display a Qt window with rgb images from a folder with their prediction from a model which uses RGB and DEPTH images.
Highlight in red the wrong prediction (error of the model)
"""

SUCCESS_THRESHOLD = 50 # A success if prediction > threshold

class PredictOnImageFilesWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(PredictOnImageFilesWindow, self).__init__(parent)
        parent_dir = QFileDialog.getExistingDirectory(self, "Select parent folder of rgb and depth folders ", "/common/work/stockage_banque_image", QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks)
        parent_dir = pathlib.Path(parent_dir)
        reply = QMessageBox.question(self, 'Choose folder to process', "Do you want to process for 'success'?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        self.is_success_dir = reply==QMessageBox.Yes
        fname = QFileDialog.getOpenFileName(self, 'Open CKPT model file', '/common/work/model_trained', "Model files (*.ckpt)",
                                            options=QFileDialog.DontUseNativeDialog)
        self.wrong = self.total = 0
        self.model = RgbAndDepthCnn.load_ckpt_model_file(fname[0])   # Load the selected model
        self.scrollArea = QtWidgets.QScrollArea(widgetResizable=True)
        self.setCentralWidget(self.scrollArea)
        content_widget = QtWidgets.QWidget()
        self.scrollArea.setWidget(content_widget)
        self._lay = QtWidgets.QVBoxLayout(content_widget)
        self.images_rgb_folder = parent_dir / 'rgb' / 'success' if self.is_success_dir else parent_dir / 'rgb' / 'fail'
        self.images_depth_folder = parent_dir / 'depth' / 'success' if self.is_success_dir else parent_dir / 'depth' / 'fail'
        self.rgb_filenames_it = iter([file for file in os.listdir(str(self.images_rgb_folder))])
        self.depth_filenames_it = iter([file for file in os.listdir(str(self.images_depth_folder))])
        self._timer = QtCore.QTimer(self, interval=1)
        self._timer.timeout.connect(self.on_timeout)
        self._timer.start()

    def on_timeout(self):
        try:
            rgb_filename = next(self.rgb_filenames_it)
            rgb_filename_with_path = self.images_rgb_folder / rgb_filename
            depth_filename = next(self.depth_filenames_it)
            depth_filename_with_path = self.images_depth_folder / depth_filename
            prob = self.compute_prediction(rgb_filename_with_path, depth_filename_with_path)
            pixmap = QtGui.QPixmap(str(rgb_filename_with_path))
            self.add_pixmap(pixmap, rgb_filename, depth_filename, prob)
        except StopIteration:
            print('fin')
            stat = f'Wrong predictions = {self.wrong} / {self.total}'
            self._lay.addWidget(QtWidgets.QLabel(stat))
            self._timer.stop()

    def compute_prediction(self, rgb_file, depth_file):
        pil_rgb_img = Image.open(rgb_file).convert('RGB')
        pil_depth_image = Image.open(depth_file).convert('RGB')
        pred = RgbAndDepthCnn.predict_from_pil_rgb_and_depth_images(self.model, pil_rgb_img, pil_depth_image)
        prob, cl = Cnn.compute_prob_and_class(pred)
        return math.floor(100 * prob)

    def add_pixmap(self, pixmap, rgb_filename, depth_filename, prob):
        if not pixmap.isNull():
            label_image = QtWidgets.QLabel(pixmap=pixmap)
            ch = f"{prob}%, RGB file = {rgb_filename}, DEPTH file = {depth_filename}"
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