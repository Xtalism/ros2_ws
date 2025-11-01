from os.path import join as joinOS
from PySide6.QtWidgets import QPushButton
from PySide6.QtGui import QPainter, QColor, QFontDatabase, QFont, QPolygon
from PySide6.QtCore import Qt, QTimer, QPoint

from ament_index_python.packages import get_package_share_directory

class RTButton(QPushButton):
    def __init__(self, text, accent, textColor, parent=None):
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
        self.customFont = QFont(families[0], 16)
        self.text = text

        self.hovered = False
        self.pressed = False
        self.glow = 30
        self.accent = accent
        self.textColor = textColor if textColor in self.colors else "fg"

        self.glowAT = QTimer(self)
        self.glowAT.timeout.connect(self.animate_glow)
        self.glowAT.start(30)

        self.setStyleSheet("background: transparent; border: none;")
        self.setFixedWidth(len(text) * 16)
        self.setMinimumSize(44, 34)

    def enterEvent(self, event):
        self.hovered = True
        self.update()
        self.setCursor(Qt.PointingHandCursor)
        super().enterEvent(event)

    def leaveEvent(self, event):
        self.hovered = False
        self.update()
        self.unsetCursor()
        super().leaveEvent(event)

    def mousePressEvent(self, event):
        self.pressed = True
        self.update()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        self.pressed = False
        self.update()
        super().mouseReleaseEvent(event)

    def animate_glow(self):
        targetGlow = 60 if self.hovered else 30
        if self.glow < targetGlow:
            self.glow += 5
        elif self.glow > targetGlow:
            self.glow -= 5
        self.update()

    def setText(self, text):
        self.text = text
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        borderWidth = 2
        
        borderColor = self.colors[self.accent[1]]
        fillColor = self.colors[self.accent[0]].darker(400 - self.glow)
        if self.pressed:
            fillColor = self.colors[self.accent[1]].darker(350 - self.glow)
        elif self.hovered:
            fillColor = self.colors[self.accent[1]].darker(600 - self.glow)

        painter.setPen(borderColor)
        painter.setBrush(fillColor)

        w, h = self.width(), self.height()
        points = [
            QPoint(0, 0),
            QPoint(w, 0),
            QPoint(w, h-16),
            QPoint(w-16, h),
            QPoint(0, h)

        ]
        painter.drawPolygon(QPolygon(points))

        painter.setFont(self.customFont)
        painter.setPen(self.colors[self.textColor])
        painter.drawText(self.rect(), Qt.AlignCenter, self.text)
