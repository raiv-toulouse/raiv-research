from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import torch
from raiv_libraries.image_tools import ImageTools
from raiv_libraries.cnn import Cnn


class CanvasExplore(QWidget):

    def __init__(self,parent):
        super().__init__(parent)
        self.parent = parent
        self.pressed = self.moving = False
        self.previous_image = None
        self.all_preds = None
        self.select_start = self.select_end = None

    def set_image(self,img):
        self.image = QImage(img.tobytes("raw", "RGB"), img.width, img.height, QImage.Format_RGB888)  # Convert PILImage to QImage
        self.setMinimumSize(self.image.width(), self.image.height())
        self.previous_image = None
        self.all_preds = None
        self.update()

    def mousePressEvent(self, event):
        modifiers = QApplication.keyboardModifiers()
        pos = event.pos()
        if event.button() == Qt.LeftButton:
            self.center = event.pos()
            if modifiers == Qt.ControlModifier:  # Ctrl + left click
                self.parent.ask_robot_to_pick(pos.x(), pos.y())
                self.parent.move_robot()
            else:  # Only left button click
                if self.previous_image:  # A new click, we restore the previous image without the rectangle
                    self.image = self.previous_image
                    self.setMinimumSize(self.image.width(), self.image.height())
                self.pressed = True
                self.update()
        elif event.button() == Qt.RightButton:
            self.select_start = pos

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton:
            self.moving = True
            self.center = event.pos()
            self.update()
        elif event.buttons() & Qt.RightButton:
            self.select_end = event.pos()
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.previous_image = self.image.copy()
            qp = QPainter(self.image)
            self._draw_rectangle(qp)
            self.pressed = self.moving = False
            self.update()
        elif event.button() == Qt.RightButton:
            if self.select_end:
                self.parent.compute_map(self.select_start, self.select_end)
                self.update()
                self.select_start = self.select_end = None

    def paintEvent(self, event):
        qp = QPainter(self)
        rect = event.rect()
        qp.drawImage(rect, self.image, rect)
        if self.moving or self.pressed:
            self._draw_rectangle(qp)
        if self.all_preds:
            self._draw_pred(qp)
        if self.select_end:
            self._draw_selected_region(qp)
        qp.end()

    def _draw_selected_region(self, qp):
        qp.setRenderHint(QPainter.Antialiasing)
        qp.setPen(QPen(Qt.blue, 5))
        w = self.select_end.x() - self.select_start.x()
        h = self.select_end.y() - self.select_start.y()
        qp.drawRect(self.select_start.x(), self.select_start.y(), w, h)

    def _draw_rectangle(self, qp):
        if self.parent.model:  # A model exists, we can do inference
            x = self.center.x() - ImageTools.CROP_WIDTH / 2
            y = self.center.y() - ImageTools.CROP_HEIGHT / 2
            qp.setRenderHint(QPainter.Antialiasing)
            qp.setPen(QPen(Qt.blue, 5))
            qp.drawPoint(self.center)
            pred = self.parent.predict_from_point(self.center.x(), self.center.y())  # calculate the prediction wih CNN
            prob, cl = Cnn.compute_prob_and_class(pred)
            is_fail = cl.item()==0
            fail_or_success = 'Fail' if is_fail else 'Success'
            text = f'{fail_or_success} : {prob*100:.1f}%'
            self.parent.lbl_pred_result.setText(text)
            qp.setPen(Qt.red if is_fail else Qt.green)
            qp.setFont(QFont('Decorative', 14))
            qp.drawText(x, y, text)
            threshold = self.parent.sb_threshold.value()
            if prob*100 > threshold:  # Success
                qp.setPen(QPen(Qt.green, 1, Qt.DashLine))
            else:  # Fail
                qp.setPen(QPen(Qt.red, 1, Qt.DashLine))
            qp.drawRect(x, y, ImageTools.CROP_WIDTH, ImageTools.CROP_HEIGHT)

    def _draw_pred(self, qp):
        """ Display all predictions (green/red point + percentage of success) from self.all_preds list """
        threshold = self.parent.sb_threshold.value()
        self.all_preds.sort(key=lambda pred: pred[2][0][1].item(), reverse=True)
        point_size_in_pixel = 3
        for idx, (x, y , pred) in enumerate(self.all_preds):
            tensor_prob,tensor_cl = torch.max(pred, 1)
            if tensor_cl.item()==0 or (tensor_cl.item()==1 and tensor_prob.item()*100 < threshold):  # Fail
                qp.setPen(QPen(Qt.red, point_size_in_pixel)) # Prediction under the threshold
            else:
                qp.setPen(QPen(Qt.green, point_size_in_pixel))# Prediction above the threshold
            if idx == 0:
                qp.setPen(QPen(Qt.blue, point_size_in_pixel))  # The best prediction
            qp.drawPoint(x, y)
            qp.setPen(Qt.black)
            prob, cl = Cnn.compute_prob_and_class(pred)
            if self.parent.cb_print_values.isChecked():
                qp.setFont(QFont('Decorative', 8))
                text = f'{prob:.1f}%'
                qp.drawText(x, y, text)



