import os

# Sprawdzenie, czy sesja jest uruchomiona na Waylandzie
if os.getenv('XDG_SESSION_TYPE') == 'xcb':
    # Ustawienie platformy Qt na 'wayland'
    os.environ['QT_QPA_PLATFORM'] = 'xcb'

import sys
import subprocess
from PyQt5 import QtWidgets, QtCore, uic
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, QProcess
from Ui.MainWindow import Ui_MainWindow
from RosClient import RosClient
from ImageProcessor import ImageProcessor


class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, *args, obj=None, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setupUi(self)

        self.ros_client = RosClient()
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

        self.vtk_process = None
        self.program_running = False

    def openVTK(self): # Funkcja obsługująca uruchamianie i zatrzymywanie wizualizacji
        if not self.program_running:
            self.startVTK()
        else:
            self.stopVTK()
    
    def startVTK(self): 
    # Metoda do rozpoczęcia procesu wizualizacji Lidaru
        self.program_running = True  # Ustawienie flagi wskazującej, że program wizualizacji jest uruchomiony
        self.vizualization_button.setText("Wyłącz Wizualizacje Lidaru")  # Zmiana tekstu przycisku na "Wyłącz Wizualizacje Lidaru"
        self.vtk_process = QProcess()  # Utworzenie obiektu procesu QProcess
        self.vtk_process.finished.connect(self.vtk_finished)  # Połączenie sygnału zakończenia procesu z metodą vtk_finished
        self.vtk_process.start("python3 lidar_visualization.py")  # Uruchomienie procesu wizualizacji Lidaru, używając polecenia 'python3 lidar_visualization.py'

    def stopVTK(self): # Funkcja zatrzymująca proces wizualizacji
        if self.vtk_process:
            self.vtk_process.kill()
            self.program_running = False
            self.vizualization_button.setText("Włącz Wizualizacje Lidaru")

    def handle_key_pressed(self, key):
        # Funkcja obsługująca naciśnięcie klawisza w czasie działania wizualizacji
        print("Pressed key:", key)  

    def vtk_finished(self, exitCode, exitStatus):
        # Funkcja wywoływana po zakończeniu procesu wizualizacji
        self.program_running = False
        self.vizualization_button.setText("Włącz Wizualizacje Lidaru")      

    def resizeEvent(self, event):
        super(MainWindow, self).resizeEvent(event)
        # Start or restart the resize timer with a delay of 1...
        # Made because otherwise app will launch with small camera until you resize manually
        self.resize_timer.start(1)

    def resize_image_label(self):
        # Resize image_label after delay
        self.image_label.resize(self.camera_frame.size())

    def image_callback(self, msg):
        cv_image = self.ros_client.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        qt_image = self.image_processor.convert_cv_to_pixmap(cv_image)
        self.display_image(qt_image)
        self.image_processor.write_frame(cv_image)

    # Metoda do wyświetlania obrazu na etykiecie obrazu (QLabel)
    # pixmap: obiekt QPixmap zawierający obraz do wyświetlenia
    def display_image(self, pixmap):
        self.image_label.setPixmap(pixmap.scaled(self.image_label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

    def toggle_camera(self):
        # Metoda do przełączania nagrywania kamery między włączonym i wyłączonym stanem
        if not self.image_processor.recording:
            # Jeśli nagrywanie nie jest włączone, uruchom nagrywanie i zmień tekst przycisku na "Stop Recording"
            self.image_processor.start_recording()
            self.record_button.setText("Stop Recording")
        else:
            # Jeśli nagrywanie jest włączone, zatrzymaj nagrywanie i zmień tekst przycisku na "Start Recording"
            self.image_processor.stop_recording()
            self.record_button.setText("Start Recording")

    def closeEvent(self, event):
        # Metoda wywoływana przy zamykaniu okna głównego aplikacji
        # Zatrzymaj nagrywanie obrazu przed zamknięciem aplikacji
        self.image_processor.stop_recording()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())