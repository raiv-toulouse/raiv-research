from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PIL import Image
from cv_bridge import CvBridge
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import *

from raiv_libraries.image_tools import ImageTools


class canvas_create_fake_dataset(QWidget):

    def __init__(self,parent):
        super().__init__(parent)
        self.parent = parent
        self.center = None

    def set_image(self, img):
        """ called by parent widget to specify a new image to display """
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(img, desired_encoding= 'passthrough')
        img = Image.fromarray(img)
        self.image = QImage(img.tobytes("raw", "RGB"), img.width, img.height, QImage.Format_RGB888)  # Convert PILImage to QImage
        self.setMinimumSize(self.image.width(), self.image.height())
        self.update()

    def mousePressEvent(self, event):
        """ when we click on the image of this canvas, store the image file in the good folder (success or fail) """
        if self.parent.image_folder:
            pos = event.pos()
            self.parent.process_click(pos.x(), pos.y()) # ask the create_image_dataset object to send the robot to this position
        else:
            QMessageBox.about(self, "Warning", "Don't forget to select an image folder")

    def mouseMoveEvent(self, event):
        self.center = event.pos()
        self.update()

    def paintEvent(self, event):
        """ Use to draw the image"""
        qp = QPainter(self)
        rect = event.rect()
        qp.drawImage(rect, self.image, rect)
        if self.center : # Draw a dashed rectangle
            x = self.center.x() - ImageTools.CROP_WIDTH / 2
            y = self.center.y() - ImageTools.CROP_HEIGHT / 2
            qp.setRenderHint(QPainter.Antialiasing)
            qp.setPen(QPen(Qt.blue, 5))
            qp.drawPoint(self.center)
            if self.parent.rb_success.isChecked():  # Success
                qp.setPen(QPen(Qt.green, 1, Qt.DashLine))
            else:  # Fail
                qp.setPen(QPen(Qt.red, 1, Qt.DashLine))
            qp.drawRect(x, y, ImageTools.CROP_WIDTH, ImageTools.CROP_HEIGHT)

