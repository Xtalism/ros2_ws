from os.path import join as joinOS
from PySide6.QtWidgets import QPushButton
from PySide6.QtGui import QPainter, QColor, QFontDatabase, QFont, QPolygon
from PySide6.QtCore import Qt, QTimer, QPoint

from ament_index_python.packages import get_package_share_directory

class RTLabel(QPushButton):
    def __init__(self, text, accent, fsize, parent=None):
        super().__init__("", parent)

        self.colors = {
            "bg": QColor(24, 24, 26),
            "fg": QColor(255, 255, 255),
            "blue": QColor(69, 161, 255),
            "teal": QColor(0, 254, 255),
            "magenta": QColor(255, 0, 255),
            "yellow": QColor(255, 233, 0),
            "green": QColor(48, 230, 11),
            "red": QColor(255, 0, 57),
            "purple": QColor(192, 105, 255)
        }

        pkgDir = joinOS(get_package_share_directory('tello_gui'), 'include')
        fontID = QFontDatabase.addApplicationFont(joinOS(pkgDir, 'fonts/RedditMono.ttf'))
        families = QFontDatabase.applicationFontFamilies(fontID)
        self.customFont = QFont(families[0], fsize)
        self.text = text
        self.accent = accent if accent in self.colors else "fg"

        self.setStyleSheet("background: transparent; border: none;")
        #self.setMinimumSize(30, fsize * 1.5)
        self.setFixedWidth(len(text) * (fsize - 2))

    def setText(self, text):
        self.text = text
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        painter.setFont(self.customFont)
        painter.setPen(self.colors[self.accent])
        painter.drawText(self.rect(), Qt.AlignCenter, self.text)
