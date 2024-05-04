import math
import time
import vtk
import os
import threading

class LidarVisualizer:
    def __init__(self, renderer):
        # Inicjalizacja renderera i aktora VTK
        self.renderer = renderer

        self.points = vtk.vtkPoints()  # Punkty do wyświetlenia
        self.vertices = vtk.vtkCellArray()  # Komórki dla punktów
        self.polyData = vtk.vtkPolyData()  # Struktura danych dla geometrii
        self.polyData.SetPoints(self.points)
        self.polyData.SetVerts(self.vertices)

        # Inicjalizacja tablicy kolorów
        self.colors = vtk.vtkUnsignedCharArray()
        self.colors.SetNumberOfComponents(3)
        self.colors.SetName("Colors")  # Ustawienie nazwy dla tablicy kolorów

        # Powiązanie tablicy kolorów z danymi punktowymi
        self.polyData.GetPointData().SetScalars(self.colors)
        
        # Ustawienie koloru tła
        self.renderer.SetBackground(0.8, 0.8, 0.8) 


        # Mapper i aktor do renderowania punktów
        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInputData(self.polyData)
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)
        
        # Ustawienia wyglądu punktów
        self.actor.GetProperty().SetPointSize(5)
        self.actor.GetProperty().SetColor(1.0, 0.0, 0.0)  # Czerwone punkty

        self.renderer.AddActor(self.actor)
                
        # Lista punktów lidaru do wyświetlenia w matplotlib
        # self.lidar_points = []

        # Inicjalizacja zmiennej przechowującej offset na osi Z
        self.z_offset = 0

        # Inicjalizacja zmiennej przechowującej czas ostatniej aktualizacji
        self.last_update_time = time.time()
        
        # Inicjalizacja pozycji poprzednich enkoderów
        self.prev_positions = [0, 0]  # Zakładamy, że enkodery 1 i 2 kontrolują pozycję kół

        
        # Lista przechowujaca pozycje punktow
        self.added_points = {}
        
        # Listy przechowujace dane z enkoderow
        
        self.wheelA = 0   
        self.wheelB = 0
        
        self.wheelA_old = 0    
        self.wheelB_old = 0
        
        self.toggle_update = True
        
    # Akcelerometry
    def update_pivot(self, orientation):
        # Wyswietlanie danych z topica akcelerometrow
        # print(orientation)  # Accelerometer 
        pass

    # Enkodery
    def update_joints(self, name, position, velocity):

        """
        wheels joints ids:
        0. back_left_wheel_joint
        1. front_left_wheel_joint
        2. front_right_wheel_joint
        3. back_right_wheel_joint
        """
        
        current_time_wheel = time.time()
        elapsed_time_wheel = current_time_wheel - self.last_update_time


        if elapsed_time_wheel >= 0.75:  # Aktualizacja co 0.5 sekundy
            self.last_update_time = current_time_wheel
            
            if self.toggle_update:
                # Aktualizacja bieżących wartości
                self.wheelA = round(position[1])
                self.wheelB = round(position[2])
                print ('New data', name[1], self.wheelA, name[2], self.wheelB )
                
            else:
                # Aktualizacja starych wartości
                self.wheelA_old = round(position[1])
                self.wheelB_old = round(position[2])
                print ('OLD DATA', name[1], self.wheelA, name[2], self.wheelB )
                
 
            # Przełączanie flagi
            self.toggle_update = not self.toggle_update

            
    def update_points(self, ranges, angle_min, angle_increment):        
        # Aktualizacja punktów na podstawie danych z lidaru
        """self.points.Reset()
        self.vertices.Reset()
        self.colors.Reset()"""

        current_time = time.time()
        elapsed_time = current_time - self.last_update_time

        if elapsed_time >= 0.75:  # Aktualizacja co 0.75 sekundy
            self.last_update_time = current_time
            self.z_offset += 0.01  # Zwiększanie wartości na osi Z o 0.1 jednostkę
        
        for i, range in enumerate(ranges):
            if range == float('inf') or range == 0.0:
                continue  # Pomijanie nieprawidłowych danych
            angle = angle_min + i * angle_increment
            x = range * math.sin(angle)  
            y = range * math.cos(angle)  
            z = - self.z_offset
            
            # Sprawdzenie, czy punkt o tych współrzędnych już istnieje
            point_key = (x, y, z)
            if point_key in self.added_points:
                # Aktualizacja koloru punktu
                self.colors.SetTuple(self.added_points[point_key], [0, 0, 255])  # Aktualizujemy kolor na niebieski
            else:
                # Dodanie nowego punktu
                pt_id = self.points.InsertNextPoint([x, y, z])
                self.vertices.InsertNextCell(1)
                self.vertices.InsertCellPoint(pt_id)

    def update_visualization(self, lidar_points, color):
        # Wyczyszczenie istniejących punktów
        # self.points.Reset()
        # self.vertices.Reset()
        # self.colors.Reset()

        # Dodanie wszystkich punktów do wizualizacji
        for point, color in zip(lidar_points, color):
            pt_id = self.points.InsertNextPoint(point)
            self.vertices.InsertNextCell(1)
            self.vertices.InsertCellPoint(pt_id)
            self.colors.InsertNextTuple(color)

        # Oznaczanie zmian w danych, aby odświeżyć wizualizację
        self.points.Modified()
        self.vertices.Modified()
        self.colors.Modified()
        self.polyData.Modified()




    def export_to_ply(self):
        current_time = time.strftime("%Y-%m-%d_%H-%M-%S")
        output_directory = os.path.expanduser("~/piper_output") # uzyskanie ścieżka do katalogu domowego użytkownika
        filename = os.path.join(output_directory, f"{current_time}_pointcloud.ply")
        os.makedirs(output_directory, exist_ok=True) # spr czy dany folder istnieje
        writer = vtk.vtkPLYWriter()
        writer.SetFileName(filename)
        writer.SetInputData(self.polyData)
        writer.SetFileTypeToASCII()
        writer.SetColorModeToDefault()
        writer.SetArrayName("Colors")
        writer.Write()


