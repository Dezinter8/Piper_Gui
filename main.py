import os

# Sprawdzenie, czy sesja jest uruchomiona na Waylandzie
if os.getenv('XDG_SESSION_TYPE') == 'wayland':
    # Ustawienie platformy Qt na 'wayland'
    os.environ['QT_QPA_PLATFORM'] = 'xcb'

import sys
import subprocess
import numpy as np
import cv2
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import vtk
import rclpy
from threading import Thread

from PyQt5 import QtWidgets, QtCore, uic
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, QProcess

from Ui.MainWindow import Ui_MainWindow
from RosClient import ImageSubscriber, LidarSubscriber, RosClient
from ImageProcessor import ImageProcessor
from lidar_visualization import LidarVisualizer


class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, *args, obj=None, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setupUi(self)

        self.image_client = ImageSubscriber()
        self.image_processor = ImageProcessor()
        self.image_format = None 

        self.image_client.image_received.connect(self.image_callback)
        self.image_label = QtWidgets.QLabel(self.camera_frame)
        self.image_label.resize(self.camera_frame.size())

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.image_client.spin_once)
        # self.timer.timeout.connect(self.updateVTK)

        self.timer.start(10)

        # self.addVTKWidget()



    def addVTKWidget(self):
        # Konfiguracja widgetu VTK do wyświetlania wizualizacji.
        self.vtkWidget = QVTKRenderWindowInteractor(self.vtk_frame)
        self.vtkWidget.setMinimumSize(200, 150)

        self.renderer = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.renderer)

        # Ustawienie wizualizera lidaru i kamery.
        self.lidarVisualizer = LidarVisualizer(self.renderer)
        self.vtkWidget.Initialize()

        camera = self.renderer.GetActiveCamera()
        camera.Zoom(0.5)
        camera.SetPosition(0, 0, 15)
        self.verticalLayout.addWidget(self.vtkWidget)

        # Inicjalizacja i uruchomienie wątku dla ROS2.
        rclpy.init()
        self.lidarSubscriber = LidarSubscriber(self.lidarVisualizer)
        self.rclpyThread = Thread(target=rclpy.spin, args=(self.lidarSubscriber,), daemon=True)
        self.rclpyThread.start()
    
    def updateVTK(self):
        # Renderowanie sceny VTK.
        self.vtkWidget.GetRenderWindow().Render()



    def image_callback(self, msg):
        # Dekompresuj obraz
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Ustawienie formatu obrazu, jeśli nie jest jeszcze ustawiony
        if self.image_format is None:
            self.image_format = cv_image.shape[2]

        # Konwersja obrazu do QImage i wyświetlenie
        qt_image = self.image_processor.convert_cv_to_pixmap(cv_image)
        self.display_image(qt_image)
        self.image_processor.write_frame(cv_image)


    def display_image(self, pixmap):
        self.image_label.setPixmap(pixmap.scaled(self.image_label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))



    def closeEvent(self, event):
        super().closeEvent(event)



if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
