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
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas


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
        # Inicjalizacja ramki na wykres matplotlib
        self.init_matplotlib()

        '''
        Utworzenie RosClient z przekazaniem funkcji:
            - obsługi obrazu 
            - wizualizatora lidaru
            - enkoderow
            - akcelerometru
        '''
        self.ros_client = RosClient(self, self.lidarVisualizer, self.image_callback, self.enkoders,  self.imu)

        # Timer do odświeżania wizualizacji VTK.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateVTK)
        self.timer.start(10)

        # Setup QLabel for displaying images
        self.image_label = QtWidgets.QLabel(self.camera_frame)
        self.image_label.setMinimumSize(531,372)

        # QTimer for delayed resizing
        self.resize_timer = QTimer(self)
        self.resize_timer.setSingleShot(True)  # Ensure timer runs only once per resize event
        self.resize_timer.timeout.connect(self.resize_image_label)


        self.record_button = self.record_button
        self.record_button.clicked.connect(self.toggle_camera)

        self.reset_vtk_view_button = self.reset_vtk_view_button
        self.reset_vtk_view_button.clicked.connect(self.resetCamera) # Połączenie przycisku z metodą openVTK
        
        self.reset_visualization_button = self.reset_visualization_button
        self.reset_visualization_button.clicked.connect(self.reset_vtk_visualization) # Połączenie przycisku z metodą reset_vtk_visualization

        self.save_pointcloud_button = self.save_pointcloud_button
        self.save_pointcloud_button.clicked.connect(self.save_pointcloud) # Połączenie przycisku z metodą save_pointcloud
        self.is_saving_pointcloud = False  # Flaga wskazująca, czy zapisywanie chmury punktów jest w toku


        self.ros_client.data_updated.connect(self.update_pivot_ui)
        self.ros_client.joints_updated.connect(self.update_joints_ui)


    ########### EXPORT CHMURY PUNKTÓW #############
        
    def save_pointcloud(self):
        if not self.is_saving_pointcloud:
            self.is_saving_pointcloud = True
            self.save_pointcloud_button.setText("Zakończ zapisywanie\nchmury punktów")
            # Rozpocznij zapisywanie chmury punktów
            self.lidarVisualizer.export_to_ply()
        else:
            self.is_saving_pointcloud = False
            self.save_pointcloud_button.setText("Rozpocznij zapisywanie\nchmury punktów")
            # Zakończ zapisywanie chmury punktów
            self.lidarVisualizer.points.Reset()
            self.lidarVisualizer.vertices.Reset()
            self.lidarVisualizer.colors.Reset()

            self.ros_client.reset_visualization()



    ########### RESET POZYCJI KÓŁ #############

    def reset_vtk_visualization(self):
        self.lidarVisualizer.points.Reset()
        self.lidarVisualizer.vertices.Reset()
        self.lidarVisualizer.colors.Reset()


        self.ros_client.reset_visualization()


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
        
        # Utworzenie i skonfigurowanie polaczenia enkoderow.
        self.enkoders = LidarVisualizer(self.renderer)
        
        # Utworzenie i skonfigurowanie poloczenia akcelerometrow.
        self.imu = LidarVisualizer(self.renderer)
        
        # Inicjalizacja widżetu VTK.
        self.vtkWidget.Initialize()

        # Ustawienia kamery w scenie VTK.
        camera = self.renderer.GetActiveCamera()
        camera.Zoom(2)
        camera.SetPosition(5, -5, 3)
        camera.SetViewUp(0, 0, 1)

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

        # Aktualizacja wykresu matplotlib
        lidar_points = self.ros_client.get_lidar_points()
        self.ax.clear()
        if lidar_points:
            lidar_points = np.array(lidar_points)
            self.ax.scatter(lidar_points[:, 0], lidar_points[:, 1], color='r', s=5)  # Wyświetlenie punktów lidaru
        self.canvas.draw()

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









    ########## REALTIME ##########

    def init_matplotlib(self):
        # Tworzenie obiektu figury Matplotlib
        self.figure = plt.figure()

        # Tworzenie obiektu Canvas z wykorzystaniem FigureCanvas
        self.canvas = FigureCanvas(self.figure)

        # Dodawanie Canvas do ramki
        layout = QtWidgets.QVBoxLayout(self.realtime_frame)
        layout.addWidget(self.canvas)

        # Tworzenie przykładowego wykresu
        self.ax = self.figure.add_subplot(111)
        self.ax.set_aspect('equal', adjustable='box')  # Ustawienie proporcji 1:1
        self.ax.plot([1, 2, 3, 4], [10, 20, 25, 30])  # Przykładowy wykres

        # Odświeżanie wyświetlania
        self.canvas.draw()





    ############# APP ############

    def keyPressEvent(self, event):
        super(MainWindow, self).keyPressEvent(event)  # Przekaż zdarzenie do bazowej klasy, jeśli nie jest obsługiwane tutaj
        
        if event.key() == QtCore.Qt.Key_R:  # Sprawdź, czy naciśnięto klawisz 'r'
            self.resetCamera()


    def closeEvent(self, event):
        self.image_processor.stop_recording()
        #export chmury punktów
        #self.lidarVisualizer.export_to_ply()

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


    def update_pivot_ui(self, data):
        self.vel_z_angle_label.setText(f"{data['vel_angle_z']:.2f}°")
        self.acc_x_angle_label.setText(f"{data['acc_angle_x']:.2f}°")
        self.acc_y_angle_label.setText(f"{data['acc_angle_y']:.2f}°")
        self.acc_z_angle_label.setText(f"{data['acc_angle_z']:.2f}°")

    def update_joints_ui(self, data):
        self.wheel_z_angle_label.setText(f"{round(data['wheel_angle_z'], 2)}°")




if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
