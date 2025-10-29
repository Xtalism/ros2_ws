#!/usr/bin/env python3
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout
from PySide6.QtCore import Qt, QRectF, QTimer, Signal, Slot
from PySide6.QtGui import QPainter, QColor, QBrush, QLinearGradient, QColor, QPixmap, QImage
from PySide6.QtWidgets import QLabel
from rtgui_button import RTButton
from rtgui_label import RTLabel
from rtgui_overlay import Overlay
from rtgui_videoframe import VideoFrame

import cv2
from os.path import join as joinOS

from ament_index_python.packages import get_package_share_directory
from rtros import RosNode, RosThread

class MainWindow(QWidget):
    keyPressed = Signal(str)
    mouseMoved = Signal(str)
    mouseScrolled = Signal(str)
    def __init__(self, title):
        super().__init__()
        self.setWindowTitle(title)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Window)
        self.setAttribute(Qt.WA_TranslucentBackground)

        self.exitBTN = RTButton('x', ['bg', 'red'], 'red')
        self.maxBTN = RTButton('▫', ['bg', 'fg'], 'yellow')
        self.minBTN = RTButton('_', ['bg', 'fg'], 'yellow')
        self.titleLBL = RTLabel(title, 'fg', 14)

        self.titleLayout = QHBoxLayout()
        self.titleLayout.addWidget(self.exitBTN)
        self.titleLayout.addWidget(self.maxBTN)
        self.titleLayout.addWidget(self.minBTN)
        self.titleLayout.addStretch()
        self.titleLayout.addWidget(self.titleLBL)
        self.titleLayout.addStretch()

        self.sensorLayout = QVBoxLayout()

        odomLabel = [
            RTLabel('Posición', 'fg', 12),
            RTLabel('X: ', 'blue', 10),
            RTLabel('Y: ', 'blue', 10),
            RTLabel('Z: ', 'blue', 10),
            RTLabel('Orientación', 'fg', 12),
            RTLabel('X: ', 'blue', 10),
            RTLabel('Y: ', 'blue', 10),
            RTLabel('Z: ', 'blue', 10),
            RTLabel('W: ', 'blue', 10)
        ]
        sensorLabel = [
            RTLabel('Batería', 'fg', 12),
            RTLabel('Cámara', 'fg', 12),
            RTLabel('Estado', 'fg', 12),
            RTLabel('Temperatura', 'fg', 12),
            RTLabel('WiFi', 'fg', 12)
        ]
        self.odomValues = [
            RTLabel(' ', 'fg', 10),
            RTLabel('0.0', 'teal', 10), 
            RTLabel('0.0', 'teal', 10), 
            RTLabel('0.0', 'teal', 10),
            RTLabel(' ', 'fg', 10),
            RTLabel('0.0', 'teal', 10), 
            RTLabel('0.0', 'teal', 10), 
            RTLabel('0.0', 'teal', 10),
            RTLabel('0.0', 'teal', 10)
        ]
        self.sensorValues = [
            RTLabel('0% [0.0V]', 'teal', 10),
            RTLabel('0x0 [0 fps]', 'teal', 10),
            RTLabel('Null', 'teal', 10),
            RTLabel('0.0°C [0.0°F]', 'teal', 10),
            RTLabel('0%', 'teal', 10)
        ]

        for i in range(9):
            imuLayout = QHBoxLayout()
            imuLayout.addWidget(odomLabel[i])
            imuLayout.addWidget(self.odomValues[i])
            imuLayout.addStretch()
            self.sensorLayout.addLayout(imuLayout)
        
        for i in range(5):
            sensorLayout = QHBoxLayout()
            sensorLayout.addWidget(sensorLabel[i])
            sensorLayout.addStretch()
            self.sensorLayout.addLayout(sensorLayout)

            sensorLayout = QHBoxLayout()
            sensorLayout.addWidget(self.sensorValues[i])
            sensorLayout.addStretch()
            self.sensorLayout.addLayout(sensorLayout)

        self.sensorLayout.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        
        self.infoLBL1 = QHBoxLayout()
        infoLBL1_1 = RTLabel('Usa las teclas', 'yellow', 10)
        infoLBL1_2 = RTLabel(' W A S D ', 'teal', 10)
        infoLBL1_3 = RTLabel('para moverte.', 'yellow', 10)
        self.infoLBL1.addStretch()
        self.infoLBL1.addWidget(infoLBL1_1)
        self.infoLBL1.addWidget(infoLBL1_2)
        self.infoLBL1.addWidget(infoLBL1_3)
        self.infoLBL1.addStretch()

        self.infoLBL2 = QHBoxLayout()
        infoLBL2_1 = RTLabel('Usa el', 'yellow', 10)
        infoLBL2_2 = RTLabel('scroll del ratón', 'teal', 10)
        infoLBL2_3 = RTLabel('para subir/bajar.', 'yellow', 10)
        self.infoLBL2.addStretch()
        self.infoLBL2.addWidget(infoLBL2_1)
        self.infoLBL2.addWidget(infoLBL2_2)
        self.infoLBL2.addWidget(infoLBL2_3)
        self.infoLBL2.addStretch()

        self.infoLBL3 = QHBoxLayout()
        infoLBL3_1 = RTLabel('Usa el', 'yellow', 10)
        infoLBL3_2 = RTLabel('ratón', 'teal', 10)
        infoLBL3_3 = RTLabel('para girar.', 'yellow', 10)
        self.infoLBL3.addStretch()
        self.infoLBL3.addWidget(infoLBL3_1)
        self.infoLBL3.addWidget(infoLBL3_2)
        self.infoLBL3.addWidget(infoLBL3_3)
        self.infoLBL3.addStretch()

        self.infoLBL4 = QHBoxLayout()
        infoLBL5_1 = RTLabel('Presiona el', 'yellow', 10)
        infoLBL5_2 = RTLabel('botón derecho (button 2)', 'teal', 10)
        infoLBL5_3 = RTLabel('para despegar/aterrizar.', 'yellow', 10)
        self.infoLBL4.addStretch()
        self.infoLBL4.addWidget(infoLBL5_1)
        self.infoLBL4.addWidget(infoLBL5_2)
        self.infoLBL4.addWidget(infoLBL5_3)
        self.infoLBL4.addStretch()

        infoLayout = QVBoxLayout()
        infoLayout.addLayout(self.infoLBL1)
        infoLayout.addLayout(self.infoLBL2)
        infoLayout.addLayout(self.infoLBL3)
        infoLayout.addLayout(self.infoLBL4)

        self.videoFrame = VideoFrame()
        self.videoFrame.setPixmap(QPixmap(1280, 720))
        self.videoFrame.pixmap().fill(QColor(0, 0, 0, 255))


        self.videoLayout = QVBoxLayout()
        self.videoLayout.addWidget(self.videoFrame)
        self.videoLayout.addStretch()
        self.videoLayout.addLayout(infoLayout)

        self.bodyLayout = QHBoxLayout()
        self.bodyLayout.addLayout(self.sensorLayout)
        self.bodyLayout.addLayout(self.videoLayout)

        mainLayout = QVBoxLayout()
        mainLayout.addLayout(self.titleLayout)
        mainLayout.addLayout(self.bodyLayout)
        
        self.bodyLayout.setContentsMargins(10, 0, 10, 10)
        mainLayout.setContentsMargins(0, 0, 0, 0)

        self.setLayout(mainLayout)

        self.exitBTN.clicked.connect(self.close)
        self.maxBTN.clicked.connect(self.toggleMaxRestore)
        self.minBTN.clicked.connect(self.showMinimized)

        pkgDir = joinOS(get_package_share_directory('tello_gui'), 'include')
        testIMG = cv2.imread(joinOS(pkgDir, 'testimg.jpg'))
        self.videoFrame.updateImage(testIMG)

        self.overlay = Overlay(self)
        self.overlay.resize(self.size())
        self.overlay.raise_()
        
        self.show()

        self.ros = RosThread()
        self.ros.start()

        self.keyPressed.connect(self.ros.node.publish)
        self.mouseMoved.connect(self.ros.node.publish)
        self.mouseScrolled.connect(self.ros.node.publish)

    def keyPressEvent(self, event):
        keyMAP = {
            Qt.Key_W: 'W',
            Qt.Key_A: 'A',
            Qt.Key_S: 'S',
            Qt.Key_D: 'D',
        }
        keyCode = event.key()

        if keyCode in keyMAP:
            key = keyMAP[keyCode]
            modifiers = event.modifiers()

            mod = ''
            if modifiers & Qt.ControlModifier:
                mod = 'CTRL'
            if modifiers & Qt.AltModifier:
                mod = 'ALT'

            if mod:
                msg = f'KEY_{mod}_{key}'
            else:
                msg = f'KEY_{key}'

            self.keyPressed.emit(msg)

        super().keyPressEvent(event)

    def wheelEvent(self, event):
        delta = int(event.angleDelta().y() / 120)

        if delta != 0:
            msg = f'SCROLL_{delta}'
            self.mouseScrolled.emit(msg)

    def mousePressEvent(self, event):
        if event.button() == Qt.RightButton:
            self.keyPressed.emit('MOUSE_2')
        elif event.button() == Qt.LeftButton:
            self.sendPos(event)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        self.sendPos(event)
        super().mouseMoveEvent(event)

    def sendPos(self, event):
        pos = event.position()
        center = self.width() / 2
        delta = int(pos.x() - center) // 4

        if delta != 0:
            msg = f'GYRO_{delta}'
            self.mouseMoved.emit(msg)

    def toggleMaxRestore(self):
        if self.isMaximized():
            self.showNormal()
            self.maxBTN.setText('▫')
        else:
            self.showMaximized()
            self.maxBTN.setText('▪')

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        grad = QLinearGradient(0, 0, 0, self.height())
        grad.setColorAt(0, QColor(0, 0, 0, 150))
        painter.setBrush(QBrush(grad))
        painter.setPen(Qt.NoPen)
        painter.drawRect(self.rect())

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.overlay.resize(self.size())

    def closeEvent(self, event):
        RosThread.stop(self.ros)
        event.accept()


if __name__ == '__main__':
    app = QApplication([])
    window = MainWindow('DJI TELLO GUI')
    window.show()
    app.exec()
