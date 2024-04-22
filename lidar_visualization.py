import math
import time
import vtk

class LidarVisualizer:
    def __init__(self, renderer):
        # Inicjalizacja renderera i aktora VTK
        self.renderer = renderer

        self.points = vtk.vtkPoints()  # Punkty do wyświetlenia
        self.vertices = vtk.vtkCellArray()  # Komórki dla punktów
        self.polyData = vtk.vtkPolyData()  # Struktura danych dla geometrii
        self.polyData.SetPoints(self.points)
        self.polyData.SetVerts(self.vertices)

        self.colors = vtk.vtkUnsignedCharArray()
        self.colors.SetNumberOfComponents(3)
        self.colors.SetName("Colors")  # Ustawienie nazwy dla tablicy kolorów

        self.polyData.GetPointData().SetScalars(self.colors)

        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInputData(self.polyData)
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)
        
        self.actor.GetProperty().SetPointSize(5)
        self.actor.GetProperty().SetColor(1.0, 0.0, 0.0)  # Czerwone punkty

        self.renderer.AddActor(self.actor)

        self.z_offset = 0

        self.last_update_time = time.time()

    def update_points(self, ranges, angle_min, angle_increment):        
        for i, range in enumerate(ranges):
            if range == float('inf') or range == 0.0:
                continue  # Pomijanie nieprawidłowych danych
            angle = angle_min + i * angle_increment
            x = range * math.sin(angle)  
            y = range * math.cos(angle)  
            z = 0
            
 
            # Dodanie nowego punktu
            pt_id = self.points.InsertNextPoint([x, y, z])
            self.vertices.InsertNextCell(1)
            self.vertices.InsertCellPoint(pt_id)

            # Wybór koloru punktu na podstawie kąta
            if i == 0:
                self.colors.InsertNextTuple([0, 255, 0])  # Zielony kolor dla punktu o kącie 0 stopni
            elif i == 167:
                self.colors.InsertNextTuple([255, 255, 0])  # Żółty kolor dla punktu o kącie 90 stopni
            elif i == 333:
                self.colors.InsertNextTuple([0, 255, 255])  # Cyan kolor dla punktu o kącie 180 stopni
            elif i == 500:
                self.colors.InsertNextTuple([255, 0, 255])  # Magenta kolor dla punktu o kącie 270 stopni
            else:
                self.colors.InsertNextTuple([255, 0, 0])  # Domyślny kolor czerwony
        
        # Oznaczanie zmian w danych, aby odświeżyć wizualizację
        self.points.Modified()
        self.vertices.Modified()
        self.polyData.Modified()


