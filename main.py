import os

# Sprawdzenie, czy sesja jest uruchomiona na Waylandzie
if os.getenv('XDG_SESSION_TYPE') == 'xcb':
    # Ustawienie platformy Qt na 'xcb'
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

        # Połączenie sygnału odbioru obrazu ROS z funkcją przetwarzania
        self.ros_client.image_received.connect(self.image_callback)

        # Ustawienie QLabel do wyświetlania obrazów
        self.image_label = QtWidgets.QLabel(self.camera_frame)
        self.image_label.resize(self.camera_frame.size())

        # Ustawienie QTimer do regularnej aktualizacji
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.ros_client.spin_once)
        self.timer.start(10)

        # QTimer do opóźnionego zmiany rozmiaru
        self.resize_timer = QTimer(self)
        self.resize_timer.setSingleShot(True)  # Zapewnia, że timer uruchamia się tylko raz podczas zdarzenia zmiany rozmiaru
        self.resize_timer.timeout.connect(self.resize_image_label)

        self.record_button = self.record_button
        self.record_button.clicked.connect(self.toggle_camera)

        self.vizualization_button.clicked.connect(self.openVTK) # Połączenie przycisku z metodą openVTK
        self.vizualization_button.setText("Włącz Wizualizacje Lidaru")

        self.vtk_process = None
        self.program_running = False

    def openVTK(self): # Metoda obsługująca uruchamianie i zatrzymywanie wizualizacji
        if not self.program_running:
            self.startVTK()
        else:
            self.stopVTK()
    
    def startVTK(self): 
        # Rozpoczęcie procesu wizualizacji Lidaru
        self.program_running = True  # Ustawienie flagi wskazującej, że program wizualizacji jest uruchomiony
        self.vizualization_button.setText("Wyłącz Wizualizacje Lidaru")  # Zmiana tekstu przycisku na "Wyłącz Wizualizacje Lidaru"
        self.vtk_process = QProcess()  # Utworzenie obiektu procesu QProcess
        self.vtk_process.finished.connect(self.vtk_finished)  # Połączenie sygnału zakończenia procesu z metodą vtk_finished
        self.vtk_process.start("python3 lidar_visualization.py")  # Uruchomienie procesu wizualizacji Lidaru, używając polecenia 'python3 lidar_visualization.py'

    def stopVTK(self): # Metoda zatrzymująca proces wizualizacji
        if self.vtk_process:
            self.vtk_process.kill()
            self.program_running = False
            self.vizualization_button.setText("Włącz Wizualizacje Lidaru")

    def handle_key_pressed(self, key):
        # Metoda obsługująca naciśnięcie klawisza podczas działania wizualizacji
        print("Naciśnięty klawisz:", key)  

    def vtk_finished(self, exitCode, exitStatus):
        # Metoda wywoływana po zakończeniu procesu wizualizacji
        self.program_running = False
        self.vizualization_button.setText("Włącz Wizualizacje Lidaru")      

    def resizeEvent(self, event):
        super(MainWindow, self).resizeEvent(event)
        # Uruchomienie lub ponowne uruchomienie timera zmiany rozmiaru z opóźnieniem 1...
        # Zrobione, ponieważ w przeciwnym razie aplikacja uruchomi się z małą kamerą, dopóki nie zmienisz rozmiaru ręcznie
        self.resize_timer.start(1)

    def resize_image_label(self):
        # Zmiana rozmiaru image_label po opóźnieniu
        self.image_label.resize(self.camera_frame.size())

    def image_callback(self, msg):
        # Konwersja obrazu ROS na obiekt obrazu OpenCV
        cv_image = self.ros_client.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Konwersja obrazu OpenCV na obiekt QPixmap i wyświetlenie
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
