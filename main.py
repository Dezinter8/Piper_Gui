import os

# Sprawdzenie, czy sesja jest uruchomiona na Waylandzie
if os.getenv('XDG_SESSION_TYPE') == 'wayland':
    # Ustawienie platformy Qt na 'wayland'
    os.environ['QT_QPA_PLATFORM'] = 'wayland'

import sys
import subprocess
from PyQt5 import QtWidgets, QtCore, uic
from PyQt5.QtCore import QTimer, QThread, pyqtSignal

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



        self.program_running = False
        
        self.vizualization_button.clicked.connect(self.openVTK)
        self.vizualization_button.setText("Wizualizacja Lidaru")
        self.teleop_thread = TeleopThread()
        self.teleop_thread.key_pressed.connect(self.handle_key_pressed)



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




    def openVTK(self):
        if not self.program_running:
            self.program_running = True
            vtk_process = subprocess.Popen(["python3", "lidar_visualization.py"])
            self.vizualization_button.setEnabled(False)
            self.teleop_thread.start()

    def handle_key_pressed(self, key):
        print("Pressed key:", key)  

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())