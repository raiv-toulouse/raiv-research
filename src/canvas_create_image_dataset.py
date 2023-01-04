from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from raiv_libraries.get_coord_node import InBoxCoord
from raiv_libraries.image_tools import ImageTools


class canvas_create_image_dataset(QWidget):

    def __init__(self,parent):
        super().__init__(parent)
        self.parent = parent
        self.setMouseTracking(True)

    def set_image(self, msg_img):
        """ called by parent widget to specify a new image to display """
        self.image = ImageTools.ros_msg_to_QImage(msg_img)
        self.setMinimumSize(self.image.width(), self.image.height())
        self.update()


    def mousePressEvent(self, event):
        """ when we click on the image of this canvas, send the robot to this position and store the image file in the good folder (success or fail) """
        pos = event.pos()
        self.parent.process_click(pos.x(), pos.y()) # ask the create_image_dataset object to send the robot to this position


    def mouseMoveEvent(self, event):
        response_from_coord_service = self.parent.coord_service('fixed', InBoxCoord.PICK, InBoxCoord.IN_THE_BOX, ImageTools.CROP_WIDTH, ImageTools.CROP_HEIGHT, event.x(), event.y())
        self.parent.canvas_preview.update_image(response_from_coord_service.rgb_crop)


    def paintEvent(self, event):
        """ Use to draw the image"""
        qp = QPainter(self)
        rect = event.rect()
        qp.drawImage(rect, self.image, rect)
