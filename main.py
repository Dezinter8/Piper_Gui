import os

# Sprawdzenie, czy sesja jest uruchomiona na Waylandzie
if os.getenv('XDG_SESSION_TYPE') == 'wayland':
    # Ustawienie platformy Qt na 'wayland'
    os.environ['QT_QPA_PLATFORM'] = 'xcb'

import sys
import numpy as np
import cv2
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import vtk

from PyQt5 import QtWidgets, QtCore, uic
from PyQt5.QtCore import QTimer

from Ui.MainWindow import Ui_MainWindow
from RosClient import RosClient
from ImageProcessor import ImageProcessor
from lidar_visualization import LidarVisualizer


class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, *args, obj=None, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setupUi(self)
        # self.lidarVisualizer = LidarVisualizer(self.renderer)

        self.image_format = None 
        self.image_processor = ImageProcessor()  # Asumując, że ImageProcessor został już zaimportowany.

        # Konfiguracja GUI z widżetami.
        self.addVTKWidget()

        # Utworzenie RosClient z przekazaniem funkcji obsługi obrazu i wizualizatora lidaru
        self.ros_client = RosClient(self.lidarVisualizer, self.image_callback)

        # Timer do odświeżania wizualizacji VTK.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateVTK)
        self.timer.start(10)

        # Setup QLabel for displaying images
        self.image_label = QtWidgets.QLabel(self.camera_frame)
        self.image_label.resize(self.camera_frame.size())

        # QTimer for delayed resizing
        self.resize_timer = QTimer(self)
        self.resize_timer.setSingleShot(True)  # Ensure timer runs only once per resize event
        self.resize_timer.timeout.connect(self.resize_image_label)


        self.record_button = self.record_button
        self.record_button.clicked.connect(self.toggle_camera)

        self.record_button = self.reset_vtk_view_button
        self.reset_vtk_view_button.clicked.connect(self.resetCamera) # Połączenie przycisku z metodą openVTK


    ########### VTK #############

    def addVTKWidget(self):
        # Konfiguracja widgetu VTK do wyświetlania wizualizacji.
        self.vtkWidget = QVTKRenderWindowInteractor(self.vtk_frame)
        self.vtkWidget.setMinimumSize(100, 100)

        # Utworzenie renderera VTK.
        self.renderer = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.renderer)

        # Utworzenie i skonfigurowanie wizualizera lidaru.
        self.lidarVisualizer = LidarVisualizer(self.renderer)

        # Inicjalizacja widżetu VTK.
        self.vtkWidget.Initialize()

        # Ustawienia kamery w scenie VTK.
        camera = self.renderer.GetActiveCamera()
        camera.Zoom(0.5)
        camera.SetPosition(0, 0, 5)

        # Ustawienie stylu interakcji na TrackballCamera.
        interactor_style = vtk.vtkInteractorStyleTrackballCamera()
        self.vtkWidget.SetInteractorStyle(interactor_style)


        # Dodanie widżetu VTK bezpośrednio do vtk_frame
        layout = QtWidgets.QVBoxLayout(self.vtk_frame)
        layout.addWidget(self.vtkWidget)
        layout.setContentsMargins(0, 0, 0, 0)
        self.vtk_frame.setLayout(layout)

    
    def updateVTK(self):
        # Renderowanie sceny VTK.
        self.vtkWidget.GetRenderWindow().Render()


    def resetCamera(self):
        if self.renderer:
            camera = self.renderer.GetActiveCamera()
            camera.SetPosition(0, 0, 2)
            camera.SetFocalPoint(0, 0, 0)
            camera.SetViewUp(0, 1, 0)
            self.vtkWidget.GetRenderWindow().Render()



    ########### CAMERA ###########

    def resizeEvent(self, event):
        super(MainWindow, self).resizeEvent(event)
        # Start or restart the resize timer with a delay of 1...
        # Made because otherwise app will launch with small camera until you resize manually
        self.resize_timer.start(1)

    def resize_image_label(self):
        # Resize image_label after delay
        self.image_label.resize(self.camera_frame.size())


    def image_callback(self, msg):
        # Dekompresja obrazu
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







    def keyPressEvent(self, event):
        super(MainWindow, self).keyPressEvent(event)  # Przekaż zdarzenie do bazowej klasy, jeśli nie jest obsługiwane tutaj
        
        if event.key() == QtCore.Qt.Key_R:  # Sprawdź, czy naciśnięto klawisz 'r'
            self.resetCamera()


    def closeEvent(self, event):
        self.image_processor.stop_recording()
        #export chmury punktów
        self.lidarVisualizer.export_to_ply()

        # Zatrzymanie timera
        if self.timer.isActive():
            self.timer.stop()

        # Zamknięcie i czyszczenie klienta ROS
        self.ros_client.shutdown()

        # Czyszczenie widżetów VTK
        self.vtkWidget.GetRenderWindow().Finalize()  # Zalecane dla czyszczenia zasobów VTK
        self.renderer.RemoveAllViewProps()  # Usunięcie wszystkich obiektów z renderera
        self.vtkWidget = None

        # Usunięcie dynamicznie utworzonych widżetów
        for widget in self.findChildren(QtWidgets.QWidget):
            widget.deleteLater()

        # Wywołanie metody bazowej
        super().closeEvent(event)







if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
