import math
import time
import vtk
import os

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
        
        # Dodanie axes do ułatwienia pracy
        self.axesActor = vtk.vtkAxesActor()
        self.axesActor.SetTotalLength(1, 1, 1)  # Ustawia długość każdej osi
        self.renderer.AddActor(self.axesActor)





    def update_visualization(self, lidar_points, color):
        # Wyczyszczenie istniejących punktów
        # self.points.Reset()
        # self.vertices.Reset()
        # self.colors.Reset()

        # Dodanie wszystkich punktów do wizualizacji
        for point, color in zip(lidar_points, color):
            if not any(math.isnan(coord) for coord in point):  # Sprawdzenie, czy punkt zawiera wartości NaN
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


