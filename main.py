import os

# Sprawdzenie, czy sesja jest uruchomiona na Waylandzie
if os.getenv('XDG_SESSION_TYPE') == 'wayland':
    # Ustawienie platformy Qt na 'wayland'
    os.environ['QT_QPA_PLATFORM'] = 'wayland'

import sys
import subprocess
import numpy as np
import cv2

from PyQt5 import QtWidgets, QtCore, uic
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, QProcess

from Ui.MainWindow import Ui_MainWindow
from RosClient import RosClient
from ImageProcessor import ImageProcessor

class TeleopThread(QThread):
    key_pressed = pyqtSignal(str)

    def run(self):
        teleop_process = subprocess.Popen(["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        while teleop_process.poll() is None:  
            output = teleop_process.stdout.readline().decode().strip()
            if output:  
                self.key_pressed.emit(output)


class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, *args, obj=None, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setupUi(self)

        self.ros_client = RosClient()
        self.image_format = None 
        self.image_processor = ImageProcessor()

        # Connect signal for ROS image reception to processing slot
        self.ros_client.image_received.connect(self.image_callback)

        # Setup QLabel for displaying images
        self.image_label = QtWidgets.QLabel(self.camera_frame)
        self.image_label.resize(self.camera_frame.size())

        # Setup QTimer for regular updates
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.ros_client.spin_once)
        self.timer.start(10)

        # QTimer for delayed resizing
        self.resize_timer = QTimer(self)
        self.resize_timer.setSingleShot(True)  # Ensure timer runs only once per resize event
        self.resize_timer.timeout.connect(self.resize_image_label)


        self.record_button = self.record_button
        self.record_button.clicked.connect(self.toggle_camera)



        self.vizualization_button.clicked.connect(self.openVTK) # Połączenie przycisku z metodą openVTK
        self.vizualization_button.setText("Włącz Wizualizacje Lidaru")
        # Inicjalizacja wątku teleoperacji i połączenie z metodą handle_key_pressed
        self.teleop_thread = TeleopThread()
        self.teleop_thread.key_pressed.connect(self.handle_key_pressed)

        self.vtk_process = None
        self.program_running = False

    def openVTK(self): # Funkcja obsługująca uruchamianie i zatrzymywanie wizualizacji
        if not self.program_running:
            self.startVTK()
        else:
            self.stopVTK()
    
    def startVTK(self): # Funkcja uruchamiająca proces wizualizacji
        self.program_running = True
        self.vizualization_button.setText("Wyłącz Wizualizacje Lidaru")
        self.vtk_process = QProcess()
        self.vtk_process.finished.connect(self.vtk_finished)
        self.vtk_process.start("python3 lidar_visualization.py")

    def stopVTK(self): # Funkcja zatrzymująca proces wizualizacji
        if self.vtk_process:
            self.vtk_process.kill()
            self.program_running = False
            self.vizualization_button.setText("Wizualizacja Lidaru")

    def handle_key_pressed(self, key):
        # Funkcja obsługująca naciśnięcie klawisza w czasie działania wizualizacji
        print("Pressed key:", key)  

    def vtk_finished(self, exitCode, exitStatus):
        # Funkcja wywoływana po zakończeniu procesu wizualizacji
        self.program_running = False
        self.vizualization_button.setText("Wizualizacja Lidaru")      



    def resizeEvent(self, event):
        super(MainWindow, self).resizeEvent(event)
        # Start or restart the resize timer with a delay of 1...
        # Made because otherwise app will launch with small camera until you resize manually
        self.resize_timer.start(1)

    def resize_image_label(self):
        # Resize image_label after delay
        self.image_label.resize(self.camera_frame.size())

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

    def toggle_camera(self):
        if not self.image_processor.recording:
            self.image_processor.start_recording()
            self.record_button.setText("Stop Recording")
        else:
            self.image_processor.stop_recording()
            self.record_button.setText("Start Recording")

    def closeEvent(self, event):
        self.image_processor.stop_recording()
        super().closeEvent(event)



if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())