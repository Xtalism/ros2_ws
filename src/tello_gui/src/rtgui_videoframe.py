from PySide6.QtWidgets import QLabel
from PySide6.QtGui import QPainter, QColor, QPixmap, QBrush, QLinearGradient, QImage
from PySide6.QtCore import Qt, QRectF

class VideoFrame(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_TranslucentBackground, False)
        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setColor(self.backgroundRole(), QColor(0, 0, 0, 255))
        self.setPalette(palette)

    def updateImage(self, image): # image is a cv2.imgread result
        height, width, channel = image.shape
        bytesPerLine = 3 * width
        qImg = QPixmap.fromImage(
            QImage(image.data, width, height, bytesPerLine, QImage.Format_RGB888).rgbSwapped()
        )
        self.setPixmap(qImg.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
