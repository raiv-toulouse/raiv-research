from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PIL import Image
from cv_bridge import CvBridge, CvBridgeError



class canvas_create_image_dataset(QWidget):

    def __init__(self,parent):
        super().__init__(parent)
        self.parent = parent

    def set_image(self, img):
        """ called by parent widget to specify a new image to display """
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(img, desired_encoding= 'passthrough')
        img = Image.fromarray(img)
        self.image = QImage(img.tobytes("raw", "RGB"), img.width, img.height, QImage.Format_RGB888)  # Convert PILImage to QImage
        print(self.image.width(),self.image.height())
        self.setMinimumSize(self.image.width(), self.image.height())
        self.update()

    def mousePressEvent(self, event):
        """ when we click on the image of this canvas, send the robot to this position and store the image file in the good folder (success or fail) """
        pos = event.pos()
        self.parent.process_click(pos.x(), pos.y()) # ask the create_image_dataset object to send the robot to this position

    def paintEvent(self, event):
        """ Use to draw the image"""
        qp = QPainter(self)
        rect = event.rect()
        qp.drawImage(rect, self.image, rect)
