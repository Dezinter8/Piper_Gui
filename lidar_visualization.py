import math
import time
import vtk
import rclpy
from threading import Thread
from RosClient import LidarSubscriber, JointStateSubscriber

class LidarVisualizer:
    def __init__(self, renderer):
        # Inicjalizacja renderera i aktora VTK
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

        # Inicjalizacja zmiennej przechowującej offset na osi Z
        self.z_offset = 0

        # Inicjalizacja zmiennej przechowującej czas ostatniej aktualizacji
        self.last_update_time = time.time()

        # Dodatkowo inicjalizujemy słownik do przechowywania już dodanych punktów
        self.added_points = {}

    def update_points(self, ranges, angle_min, angle_increment):
        # Aktualizacja punktów na podstawie danych z lidaru
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
                self.added_points[point_key] = pt_id  # Dodanie punktu do słownika

                # Wybór koloru punktu na podstawie kąta
                if i == 0:
                    self.colors.InsertNextTuple([0, 255, 0])  # Zielony kolor dla punktu o kącie 0 stopni
                elif i == 90:
                    self.colors.InsertNextTuple([255, 255, 0])  # Żółty kolor dla punktu o kącie 90 stopni
                elif i == 180:
                    self.colors.InsertNextTuple([0, 255, 255])  # Cyan kolor dla punktu o kącie 180 stopni
                elif i == 270:
                    self.colors.InsertNextTuple([255, 0, 255])  # Magenta kolor dla punktu o kącie 270 stopni
                else:
                    self.colors.InsertNextTuple([255, 0, 0])  # Domyślny kolor czerwony
        
        # Oznaczanie zmian w danych, aby odświeżyć wizualizację
        self.points.Modified()
        self.vertices.Modified()
        self.polyData.Modified()

        # Export punktów
        writer = vtk.vtkPLYWriter()
        writer.SetFileName("output.ply")
        writer.SetInputData(self.polyData)
        writer.Write()
        

def main(args=None):
    # Inicjalizacja ROS 2
    rclpy.init(args=args)

    # Inicjalizacja obiektu renderera i okna renderowania VTK
    renderer = vtk.vtkRenderer()
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.SetSize(600, 500)
    renderWindow.SetWindowName("Wizualizacja Liadru")
    renderWindow.AddRenderer(renderer)
    
    # Ustawienie okna po prawej stronie ekranu
    screen_width = renderWindow.GetScreenSize()[0]
    window_width = renderWindow.GetSize()[0]
    renderWindow.SetPosition(screen_width - window_width, 0)
    
    # Tworzenie interaktora do obsługi zdarzeń
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)


    # Inicjalizacja wizualizatora lidaru
    visualizer = LidarVisualizer(renderer)
    # Przekazanie wizualizatora do subskrybenta
    lidar_subscriber = LidarSubscriber(visualizer)

    # Inicjalizacja subskrybenta akcelerometru
    joint_state_subscriber = JointStateSubscriber()
    
    # Dodanie aktora do renderera
    renderer.AddActor(visualizer.actor)

    # Ustawienia kamery
    camera = renderer.GetActiveCamera()
    camera.Zoom(0.5)
    camera.SetPosition(0, 0, 10)

    # Ustawienie interakcji za pomocą myszki
    interactor_style = vtk.vtkInteractorStyleTrackballCamera()
    renderWindowInteractor.SetInteractorStyle(interactor_style)

    # Obsługa zdarzeń klawiatury
    def key_press(obj, event):
        key = obj.GetKeySym()
        if key == "r":
            camera.SetPosition(0, 0, 10)
            camera.SetFocalPoint(0, 0, 0)
            camera.SetViewUp(0, 1, 0)
        renderWindow.Render()

    renderWindowInteractor.AddObserver("KeyPressEvent", key_press)

    # Funkcja do cyklicznego odświeżania renderera
    def updateVTK(_obj, _event):
        renderWindow.Render()

    renderWindowInteractor.AddObserver('TimerEvent', updateVTK)
    renderWindowInteractor.CreateRepeatingTimer(100)

    # Wątek dla ROS 2
    rclpy_thread = Thread(target=rclpy.spin, args=(lidar_subscriber,), daemon=True)
    rclpy_thread.start()

    # Wyświetlenie stanu połączenia z Lidarem
    print(lidar_subscriber.ConectionStatus)
    
    # Wyświetlenie stanu połączenia z Enkoderami
    print(joint_state_subscriber.ConnectionStatus)

    # Renderowanie sceny i rozpoczęcie interakcji
    renderWindow.Render()
    renderWindowInteractor.Start()

    # Wyłączenie ROS 2
    rclpy.shutdown()

if __name__ == '__main__':
    main()
