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


from PyQt5 import QtWidgets, QtCore, uic, QtGui
from PyQt5.QtCore import QTimer

from Ui.MainWindow import Ui_MainWindow
from RosClient import RosClient
from ImageProcessor import ImageProcessor
from lidar_visualization import LidarVisualizer

import subprocess

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
        self.ros_client = RosClient(self, self.lidarVisualizer, self.image_callback, self.enkoders,  self.imu, self.update_status_labels)

        # Timer do odświeżania wizualizacji VTK.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateVTK)
        self.timer.start(10)

        # Setup QLabel for displaying images
        self.image_label = QtWidgets.QLabel(self.camera_frame)
        self.image_label.setMinimumSize(531,372)
        self.image_label.setAlignment(QtCore.Qt.AlignCenter)

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


    ########### STEROWANIE #############

        # COFANIE
        self.cofaj_button = self.cofaj_button
        self.cofaj_button.clicked.connect(self.cofaj)
        
        # DIODA
        self.dioda_button = self.dioda_button
        self.dioda_button.clicked.connect(self.dioda)
        self.dioda_is_on = False  # Flaga wskazująca, czy dioda jest uruchomiona
        
        # NAPRZÓD
        self.naprzod_button = self.naprzod_button
        self.naprzod_button.clicked.connect(self.naprzod)
        
        # STOP
        self.stop_button = self.stop_button
        self.stop_button.clicked.connect(self.stop)
        
        # W LEWO
        self.w_lewo_button = self.w_lewo_button
        self.w_lewo_button.clicked.connect(self.w_lewo)
        
        # W PRAWO
        self.w_prawo_button = self.w_prawo_button
        self.w_prawo_button.clicked.connect(self.w_prawo)
        
        # W LEWO POD KATEM DO PRZODU
        self.w_lewo_pod_katem_do_przodu_button = self.w_lewo_pod_katem_do_przodu_button
        self.w_lewo_pod_katem_do_przodu_button.clicked.connect(self.L_pod_katem_przod)
        
        # W LEWO POD KATEM DO TYLU
        self.w_lewo_pod_katem_do_tylu_button = self.w_lewo_pod_katem_do_tylu_button
        self.w_lewo_pod_katem_do_tylu_button.clicked.connect(self.L_pod_katem_tyl)

        # W PRAWO POD KATEM DO PRZODU
        self.w_prawo_pod_katem_do_przodu_button = self.w_prawo_pod_katem_do_przodu_button
        self.w_prawo_pod_katem_do_przodu_button.clicked.connect(self.P_pod_katem_przod)
         
        # W PRAWO POD KATEM DO TYLU
        self.w_prawo_pod_katem_do_tylu_button = self.w_prawo_pod_katem_do_tylu_button
        self.w_prawo_pod_katem_do_tylu_button.clicked.connect(self.P_pod_katem_tyl)

       
       
    def cofaj(self): 
        speed = self.speed_value.value()  # Pobranie wartości z QSpinBox o nazwie 'speed_value'
        
        # Ustalanie komendy na podstawie wartości 'speed'
        if speed == 1:
            self.ros_client.publish_command(2.0, 1.0, 0.0)
        elif speed == 2:
            self.ros_client.publish_command(2.0, 1.25, 0.0)
        elif speed == 3:
            self.ros_client.publish_command(2.0, 1.5, 0.0)
        elif speed == 4:
            self.ros_client.publish_command(2.0, 1.75, 0.0)
        elif speed == 5:
            self.ros_client.publish_command(2.0, 2.0, 0.0)
        elif speed == 6:
            self.ros_client.publish_command(2.0, 2.25, 0.0)
        elif speed == 7:
            self.ros_client.publish_command(2.0, 2.5, 0.0)
        elif speed == 8:
            self.ros_client.publish_command(2.0, 2.75, 0.0)
        elif speed == 9:
            self.ros_client.publish_command(2.0, 3.0, 0.0)
        else:
            return  # Zabezpieczenie na wypadek nieoczekiwanej wartości

    def dioda(self): 
        if not self.dioda_is_on:
            self.dioda_is_on = True
            self.dioda_button.setText("[L] Dioda ON ")
            self.ros_client.publish_command(16.0, 0.0, 0.0)
        else:
            self.dioda_is_on = False
            self.dioda_button.setText("[L] Dioda OFF")
            self.ros_client.publish_command(17.0, 0.0, 0.0)
        
    def naprzod(self): 
        speed = self.speed_value.value()  # Pobranie wartości z QSpinBox o nazwie 'speed_value'
        
        # Ustalanie komendy na podstawie wartości 'speed'
        if speed == 1:
            self.ros_client.publish_command(1.0, 1.0, 0.0)
        elif speed == 2:
            self.ros_client.publish_command(1.0, 1.25, 0.0)
        elif speed == 3:
            self.ros_client.publish_command(1.0, 1.5, 0.0)
        elif speed == 4:
            self.ros_client.publish_command(1.0, 1.75, 0.0)
        elif speed == 5:
            self.ros_client.publish_command(1.0, 2.0, 0.0)
        elif speed == 6:
            self.ros_client.publish_command(1.0, 2.25, 0.0)
        elif speed == 7:
            self.ros_client.publish_command(1.0, 2.5, 0.0)
        elif speed == 8:
            self.ros_client.publish_command(1.0, 2.75, 0.0)
        elif speed == 9:
            self.ros_client.publish_command(1.0, 3.0, 0.0)
        else:
            return  # Zabezpieczenie na wypadek nieoczekiwanej wartości
   
    def stop(self): 
        self.ros_client.publish_command(0.0, 0.0, 0.0)

    def w_lewo(self): 
        speed = self.speed_value.value()  # Pobranie wartości z QSpinBox o nazwie 'speed_value'
        
        # Ustalanie komendy na podstawie wartości 'speed'
        if speed == 1:
            self.ros_client.publish_command(3.0, 1.0, 0.0)
        elif speed == 2:
            self.ros_client.publish_command(3.0, 1.25, 0.0)
        elif speed == 3:
            self.ros_client.publish_command(3.0, 1.5, 0.0)
        elif speed == 4:
            self.ros_client.publish_command(3.0, 1.75, 0.0)
        elif speed == 5:
            self.ros_client.publish_command(3.0, 2.0, 0.0)
        elif speed == 6:
            self.ros_client.publish_command(3.0, 2.25, 0.0)
        elif speed == 7:
            self.ros_client.publish_command(3.0, 2.5, 0.0)
        elif speed == 8:
            self.ros_client.publish_command(3.0, 2.75, 0.0)
        elif speed == 9:
            self.ros_client.publish_command(3.0, 3.0, 0.0)
        else:
            return  # Zabezpieczenie na wypadek nieoczekiwanej wartości

    def w_prawo(self): 
        speed = self.speed_value.value()  # Pobranie wartości z QSpinBox o nazwie 'speed_value'
        
        # Ustalanie komendy na podstawie wartości 'speed'
        if speed == 1:
            self.ros_client.publish_command(4.0, 1.0, 0.0)
        elif speed == 2:
            self.ros_client.publish_command(4.0, 1.25, 0.0)
        elif speed == 3:
            self.ros_client.publish_command(4.0, 1.5, 0.0)
        elif speed == 4:
            self.ros_client.publish_command(4.0, 1.75, 0.0)
        elif speed == 5:
            self.ros_client.publish_command(4.0, 2.0, 0.0)
        elif speed == 6:
            self.ros_client.publish_command(4.0, 2.25, 0.0)
        elif speed == 7:
            self.ros_client.publish_command(4.0, 2.5, 0.0)
        elif speed == 8:
            self.ros_client.publish_command(4.0, 2.75, 0.0)
        elif speed == 9:
            self.ros_client.publish_command(4.0, 3.0, 0.0)
        else:
            return  # Zabezpieczenie na wypadek nieoczekiwanej wartości
        
    def L_pod_katem_przod(self): 
        speed = self.speed_value.value()  # Pobranie wartości z QSpinBox o nazwie 'speed_value'
        
        # Ustalanie komendy na podstawie wartości 'speed'
        if speed == 1:
            self.ros_client.publish_command(5.0, 1.0, 0.0)
        elif speed == 2:
            self.ros_client.publish_command(5.0, 1.25, 0.0)
        elif speed == 3:
            self.ros_client.publish_command(5.0, 1.5, 0.0)
        elif speed == 4:
            self.ros_client.publish_command(5.0, 1.75, 0.0)
        elif speed == 5:
            self.ros_client.publish_command(5.0, 2.0, 0.0)
        elif speed == 6:
            self.ros_client.publish_command(5.0, 2.25, 0.0)
        elif speed == 7:
            self.ros_client.publish_command(5.0, 2.5, 0.0)
        elif speed == 8:
            self.ros_client.publish_command(5.0, 2.75, 0.0)
        elif speed == 9:
            self.ros_client.publish_command(5.0, 3.0, 0.0)
        else:
            return  # Zabezpieczenie na wypadek nieoczekiwanej wartości

    def P_pod_katem_przod(self): 
        speed = self.speed_value.value()  # Pobranie wartości z QSpinBox o nazwie 'speed_value'
        
        # Ustalanie komendy na podstawie wartości 'speed'
        if speed == 1:
            self.ros_client.publish_command(6.0, 1.0, 0.0)
        elif speed == 2:
            self.ros_client.publish_command(6.0, 1.25, 0.0)
        elif speed == 3:
            self.ros_client.publish_command(6.0, 1.5, 0.0)
        elif speed == 4:
            self.ros_client.publish_command(6.0, 1.75, 0.0)
        elif speed == 5:
            self.ros_client.publish_command(6.0, 2.0, 0.0)
        elif speed == 6:
            self.ros_client.publish_command(6.0, 2.25, 0.0)
        elif speed == 7:
            self.ros_client.publish_command(6.0, 2.5, 0.0)
        elif speed == 8:
            self.ros_client.publish_command(6.0, 2.75, 0.0)
        elif speed == 9:
            self.ros_client.publish_command(6.0, 3.0, 0.0)
        else:
            return  # Zabezpieczenie na wypadek nieoczekiwanej wartości

    def L_pod_katem_tyl(self): 
        speed = self.speed_value.value()  # Pobranie wartości z QSpinBox o nazwie 'speed_value'
        
        # Ustalanie komendy na podstawie wartości 'speed'
        if speed == 1:
            self.ros_client.publish_command(7.0, 1.0, 0.0)
        elif speed == 2:
            self.ros_client.publish_command(7.0, 1.25, 0.0)
        elif speed == 3:
            self.ros_client.publish_command(7.0, 1.5, 0.0)
        elif speed == 4:
            self.ros_client.publish_command(7.0, 1.75, 0.0)
        elif speed == 5:
            self.ros_client.publish_command(7.0, 2.0, 0.0)
        elif speed == 6:
            self.ros_client.publish_command(7.0, 2.25, 0.0)
        elif speed == 7:
            self.ros_client.publish_command(7.0, 2.5, 0.0)
        elif speed == 8:
            self.ros_client.publish_command(7.0, 2.75, 0.0)
        elif speed == 9:
            self.ros_client.publish_command(7.0, 3.0, 0.0)
        else:
            return  # Zabezpieczenie na wypadek nieoczekiwanej wartości

    def P_pod_katem_tyl(self): 
        speed = self.speed_value.value()  # Pobranie wartości z QSpinBox o nazwie 'speed_value'
        
        # Ustalanie komendy na podstawie wartości 'speed'
        if speed == 1:
            self.ros_client.publish_command(8.0, 1.0, 0.0)
        elif speed == 2:
            self.ros_client.publish_command(8.0, 1.25, 0.0)
        elif speed == 3:
            self.ros_client.publish_command(8.0, 1.5, 0.0)
        elif speed == 4:
            self.ros_client.publish_command(8.0, 1.75, 0.0)
        elif speed == 5:
            self.ros_client.publish_command(8.0, 2.0, 0.0)
        elif speed == 6:
            self.ros_client.publish_command(8.0, 2.25, 0.0)
        elif speed == 7:
            self.ros_client.publish_command(8.0, 2.5, 0.0)
        elif speed == 8:
            self.ros_client.publish_command(8.0, 2.75, 0.0)
        elif speed == 9:
            self.ros_client.publish_command(8.0, 3.0, 0.0)
        else:
            return  # Zabezpieczenie na wypadek nieoczekiwanej wartości

    def increaseSpeedValue(self):
        # Zwiększanie wartości o 1
        current_value = self.speed_value.value()
        new_value = min(current_value + 1, 9)  # Zapobieganie przekroczeniu maksymalnej wartości
        self.speed_value.setValue(new_value)

    def decreaseSpeedValue(self):
        # Obniżanie wartości o 1, ale nie poniżej minimalnej wartości
        current_value = self.speed_value.value()
        if current_value > 1:
            new_value = max(current_value - 1, 1)  # Zapobieganie spadnięciu poniżej minimalnej wartości
            self.speed_value.setValue(new_value)


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



    ########### RESET POZYCJI KÓŁ #############

    def reset_vtk_visualization(self):
        self.ros_client.reset_visualization()
        
        self.lidarVisualizer.points.Reset()
        self.lidarVisualizer.vertices.Reset()
        self.lidarVisualizer.colors.Reset()



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
            camera.SetPosition(5, -5, 3)
            camera.SetFocalPoint(0, 0, 0.3)
            camera.SetViewUp(0, 0, 1)
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
            
        elif event.key() == QtCore.Qt.Key_N:   # Sprawdź, czy naciśnięto klawisz 'n'
            self.cofaj()
            
        elif event.key() == QtCore.Qt.Key_L:   # Sprawdź, czy naciśnięto klawisz 'l'
            self.dioda()
            
        elif event.key() == QtCore.Qt.Key_Y:   # Sprawdź, czy naciśnięto klawisz 'y'
            self.naprzod()
            
        elif event.key() == QtCore.Qt.Key_H:   # Sprawdź, czy naciśnięto klawisz 'h'
            self.stop()
            
        elif event.key() == QtCore.Qt.Key_G:   # Sprawdź, czy naciśnięto klawisz 'g'
            self.w_lewo()
            
        elif event.key() == QtCore.Qt.Key_J:   # Sprawdź, czy naciśnięto klawisz 'j'
            self.w_prawo()
            
        elif event.key() == QtCore.Qt.Key_T:   # Sprawdź, czy naciśnięto klawisz 't'
            self.L_pod_katem_przod()
            
        elif event.key() == QtCore.Qt.Key_U:   # Sprawdź, czy naciśnięto klawisz 'u'
            self.P_pod_katem_przod()
            
        elif event.key() == QtCore.Qt.Key_B:   # Sprawdź, czy naciśnięto klawisz 't'
            self.L_pod_katem_tyl()
            
        elif event.key() == QtCore.Qt.Key_M:   # Sprawdź, czy naciśnięto klawisz 'u'
            self.P_pod_katem_tyl()

        elif event.key() == QtCore.Qt.Key_I:   # Sprawdź, czy naciśnięto klawisz 'u'
            self.increaseSpeedValue()

        elif event.key() == QtCore.Qt.Key_K:   # Sprawdź, czy naciśnięto klawisz 'u'
            self.decreaseSpeedValue()

            
    # def keyReleaseEvent(self, event): # Zadania do wykonania przy puszczeniu klawisza
    #     super(MainWindow, self).keyReleaseEvent(event)  
        
    #     key_stop_list = [
    #         QtCore.Qt.Key_N, # Sprawdźenie klawisza 'n' - cofaj
            
    #         QtCore.Qt.Key_Y, # Sprawdźenie klawisza 'y' - naprzod
            
    #         QtCore.Qt.Key_G, # Sprawdźenie klawisza 'g' - w_lewo
            
    #         QtCore.Qt.Key_J, # Sprawdźenie klawisza 'j' - w_prawo
            
    #         QtCore.Qt.Key_T, # Sprawdźenie klawisza 't' - t_w_lewo
            
    #         QtCore.Qt.Key_U  # Sprawdźenie klawisza 'u' - t_w_prawo
    #     ]
        
    #     if event.key() in key_stop_list:
    #         self.stop() # wykonanie polecenia stop przy puszczeniu klawisza

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

    def update_joints_ui(self, data):
        self.speed_label.setText(f"{data['average_speed_mps']:.2f} m/s")


    ########### STATUS ###########

    def update_status_labels(self, camera_status, lidar_status, motors_status, imu_status):

        # Utworzenie paletek dla stanu "OK" (zielony) i "Failed" (czerwony)
        palette_green = QtGui.QPalette()
        brush_green = QtGui.QBrush(QtGui.QColor(38, 162, 105))  # Zielony
        brush_green.setStyle(QtCore.Qt.SolidPattern)
        palette_green.setBrush(QtGui.QPalette.Active, QtGui.QPalette.WindowText, brush_green)
        palette_green.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.WindowText, brush_green)
        
        palette_red = QtGui.QPalette()
        brush_red = QtGui.QBrush(QtGui.QColor(224, 27, 36))  # Czerwony
        brush_red.setStyle(QtCore.Qt.SolidPattern)
        palette_red.setBrush(QtGui.QPalette.Active, QtGui.QPalette.WindowText, brush_red)
        palette_red.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.WindowText, brush_red)


        # Aktualizacja etykiet dla kamery
        if camera_status == "OK":
            self.Camera_status_label.setPalette(palette_green)
        else:
            self.Camera_status_label.setPalette(palette_red)
        self.Camera_status_label.setText(camera_status)

        # Aktualizacja etykiet dla lidaru
        if lidar_status == "OK":
            self.Lidar_status_label.setPalette(palette_green)
        else:
            self.Lidar_status_label.setPalette(palette_red)
        self.Lidar_status_label.setText(lidar_status)

        # Aktualizacja etykiet dla IMU
        if imu_status == "OK":
            self.Imu_status_label.setPalette(palette_green)
        else:
            self.Imu_status_label.setPalette(palette_red)
        self.Imu_status_label.setText(imu_status)

        # Aktualizacja etykiet dla silników
        if motors_status == "OK":
            self.Motors_status_label.setPalette(palette_green)
        else:
            self.Motors_status_label.setPalette(palette_red)
        self.Motors_status_label.setText(motors_status)




if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
