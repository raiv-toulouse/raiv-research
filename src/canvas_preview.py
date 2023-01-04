from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from raiv_libraries.image_tools import ImageTools


class canvas_preview(QWidget):

    def __init__(self,parent):
        super().__init__(parent)
        self.parent = parent
        self.qt_img = None

    def update_image(self, img_msg):
        self.qt_img = ImageTools.ros_msg_to_QImage(img_msg)
        self.update()

    def paintEvent(self, event):
        if self.qt_img:
            qp = QPainter(self)
            rect = QRect(0, 0, ImageTools.CROP_WIDTH, ImageTools.CROP_HEIGHT)
            qp.drawImage(rect, self.qt_img, rect)
            qp.end()