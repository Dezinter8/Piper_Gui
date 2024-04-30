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

        # Mapper i aktor do renderowania punktów
        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInputData(self.polyData)
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)
        
        # Ustawienia wyglądu punktów
        self.actor.GetProperty().SetPointSize(5)
        self.actor.GetProperty().SetColor(1.0, 0.0, 0.0)  # Czerwone punkty

        self.renderer.AddActor(self.actor)

        # Inicjalizacja zmiennej przechowującej offset na osi Z
        self.z_offset = 0

        # Inicjalizacja zmiennej przechowującej czas ostatniej aktualizacji
        self.last_update_time = time.time()
        
        # Lista przechowujca pozycje punktow
        self.added_points = {}
        
        # Listy przechowujce dane z enkoderow
        self.idnr1 = 0
        self.wheelA = []    
        self.wheelB = []

    # Akcelerometry
    def update_pivot(self, orientation):
        # Wyswietlanie danych z topica akcelerometrow
        print(orientation)  # Accelerometer 

    # Enkodery
    def update_joints(self, name, position, velocity):


        self.wheelA.append([self.idnr1, name[0], position[0], velocity[0]])
        self.wheelB.append([self.idnr1, name[1], position[1], velocity[1]])
        
        print(self.wheelA[self.idnr1])  # Lewy enkoder - A
        print(self.wheelB[self.idnr1])  # Prawy enkoder - B
        
        self.idnr1 += 1
        

    def update_points(self, ranges, angle_min, angle_increment):        
        # Aktualizacja punktów na podstawie danych z lidaru
        """self.points.Reset()
        self.vertices.Reset()
        self.colors.Reset()"""
        
        current_time = time.time()
        elapsed_time = current_time - self.last_update_time

        if elapsed_time >= 0.5:  # Aktualizacja co 0.75 sekundy
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
