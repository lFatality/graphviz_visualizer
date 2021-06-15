from PySide2.QtCore import Qt, QPoint
from PySide2.QtWidgets import QWidget
from PySide2.QtGui import QPainter, QPen, QBrush, QColor, QPalette

class CircleDrawer(QWidget):
    """ The circle drawer is responsible for painting a colored circle. """

    def __init__(self, diameter=25, initial_color=Qt.green):
        super(CircleDrawer, self).__init__()
        self.setGeometry(0, 0, diameter*1.5, diameter*1.5)
        self.setMinimumWidth(diameter*1.5)
        self.setMinimumHeight(diameter*1.5)
        self.diameter_ = diameter
        self.color_ = initial_color

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.translate(self.width() / 2, self.height() / 2)
        painter.setPen(QPen(Qt.black, 3, Qt.SolidLine))
        painter.setBrush(QBrush(self.color_, Qt.SolidPattern))
        painter.drawEllipse(-self.diameter_ / 2, -self.diameter_ / 2, self.diameter_, self.diameter_)
        painter.end()