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
                
        # Listy przechowujce dane z enkoderow
        self.idnr1 = 0
        self.wheelA = []    
        self.wheelB = []

        # Lista punktów lidaru do wyświetlenia w matplotlib
        # self.lidar_points = []

        # Dodanie axes do ułatwienia pracy
        self.axesActor = vtk.vtkAxesActor()
        self.axesActor.SetTotalLength(1, 1, 1)  # Ustawia długość każdej osi
        self.renderer.AddActor(self.axesActor)


    # Enkodery
    def update_joints(self, name, position, velocity):
        # self.wheelA.append([self.idnr1, name[0], position[0], velocity[0]])
        # self.wheelB.append([self.idnr1, name[1], position[1], velocity[1]])
        
        # print(self.wheelA[self.idnr1])  # Lewy enkoder - A
        # print(self.wheelB[self.idnr1])  # Prawy enkoder - B
        
        # self.idnr1 += 1

        pass


    def update_visualization(self, lidar_points):
        # Wyczyszczenie istniejących punktów
        self.points.Reset()
        self.vertices.Reset()
        self.colors.Reset()

        # Dodanie wszystkich punktów do wizualizacji
        for point in lidar_points:
            pt_id = self.points.InsertNextPoint(point)
            self.vertices.InsertNextCell(1)
            self.vertices.InsertCellPoint(pt_id)
            self.colors.InsertNextTuple([255, 0, 0])  # Domyślny kolor czerwony

        # Oznaczanie zmian w danych, aby odświeżyć wizualizację
        self.points.Modified()
        self.vertices.Modified()
        self.polyData.Modified()




    def get_color_from_intensity(self, intensity):
        if math.isnan(intensity):  # Sprawdzenie czy intensywność jest NaN
            return [0, 0, 0]
        else:
            color_value = int(intensity)  # Skalowanie intensywności do wartości koloru (0-255)
            color = [color_value, 0, 0]  # Ustawienie RGB koloru
            return color


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

    def get_lidar_points(self):
        # return self.lidar_points
        pass
