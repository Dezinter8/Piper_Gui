import math
import vtk
import rclpy
from threading import Thread
from RosClient import LidarSubscriber

class LidarVisualizer:
    def __init__(self, renderer):
        # Inicjalizacja renderera i aktora VTK
        #self.renderer = renderer
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
        
        # Ustawienia wyglądu punktów.
        self.actor.GetProperty().SetPointSize(5)
        self.actor.GetProperty().SetColor(1.0, 0.0, 0.0)  # Czerwone punkty
        #self.renderer.AddActor(self.actor)

    def update_points(self, ranges, angle_min, angle_increment):
        # Aktualizacja punktów na podstawie danych z lidaru
        self.points.Reset()
        self.vertices.Reset()
        self.colors.Reset()

        # Indeks odpowiadający kątowi 0
        zero_angle_index = round(abs(angle_min) / angle_increment)

        for i, range in enumerate(ranges):
            if range == float('inf') or range == 0.0:
                continue  # Pomijanie nieprawidłowych danych
            angle = angle_min + i * angle_increment
            # Przesunięcie punktów tak, aby kąt zero był na dole (oś X)
            x = range * math.cos(angle - math.pi/2)  # Odejmujemy pi/2, aby obrócić kąt zero na oś X
            y = range * math.sin(angle - math.pi/2)  # Odejmujemy pi/2, aby obrócić kąt zero na oś X
            z = 0
            pt_id = self.points.InsertNextPoint([x, y, z])
            self.vertices.InsertNextCell(1)
            self.vertices.InsertCellPoint(pt_id)

            # Ustawienie koloru punktu na zielony, jeśli kąt odpowiada kątowi 0
            if i == zero_angle_index:
                self.colors.InsertNextTuple([0, 255, 0])  # Kolor zielony
            else:
                self.colors.InsertNextTuple([255, 0, 0])  # Domyślny kolor czerwony
        
        # Oznaczanie zmian w danych, aby odświeżyć wizualizację
        self.points.Modified()
        self.vertices.Modified()
        self.polyData.Modified()

def main(args=None):
    rclpy.init(args=args)

    renderer = vtk.vtkRenderer() # Tworzenie renderu
    # Tworzenie okna renderowania i dodawanie do niego renderu
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.SetSize(600, 500) # Rozmiar okna
    renderWindow.SetWindowName("Wizualizacja Liadru") # Nazwa okna renderu 
    renderWindow.AddRenderer(renderer)
    # Tworzenie interaktora i ustawianie okna renderowania
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)

    visualizer = LidarVisualizer(renderer)  # Przekazanie renderer jako argumentu
    lidar_subscriber = LidarSubscriber(visualizer)

    renderer.AddActor(visualizer.actor) # Dodanie aktora (punktów)

    camera = renderer.GetActiveCamera()
    camera.Zoom(0.5)  
    camera.SetPosition(0, 0, 10)

    # Ustawienie interactor style na vtkInteractorStyleTrackballCamera 
    interactor_style = vtk.vtkInteractorStyleTrackballCamera()
    renderWindowInteractor.SetInteractorStyle(interactor_style)
    
        # Function to handle keypress events
    def key_press(obj, event):
        key = obj.GetKeySym()
        if key == "i":
            camera.Elevation(5)
        elif key == "k":
            camera.Elevation(-5)
        elif key == "j":
            camera.Azimuth(5)
        elif key == "l":
            camera.Azimuth(-5)
        elif key == "u" or key == "W":
            camera.SetPosition(camera.GetPosition()[0] + 0.1 * camera.GetViewPlaneNormal()[0],
                               camera.GetPosition()[1] + 0.1 * camera.GetViewPlaneNormal()[1],
                               camera.GetPosition()[2] + 0.1 * camera.GetViewPlaneNormal()[2])
        elif key == "o" or key == "S":
            camera.SetPosition(camera.GetPosition()[0] - 0.1 * camera.GetViewPlaneNormal()[0],
                               camera.GetPosition()[1] - 0.1 * camera.GetViewPlaneNormal()[1],
                               camera.GetPosition()[2] - 0.1 * camera.GetViewPlaneNormal()[2])
        renderWindow.Render()

    renderWindowInteractor.AddObserver("KeyPressEvent", key_press)
    
    def updateVTK(_obj, _event): # Funkcja aktualizująca obraz VTK
        renderWindow.Render()

    renderWindowInteractor.AddObserver('TimerEvent', updateVTK)
    renderWindowInteractor.CreateRepeatingTimer(100)

    rclpy_thread = Thread(target=rclpy.spin, args=(lidar_subscriber,), daemon=True)
    rclpy_thread.start()

    renderWindow.Render() # Renderowanie sceny VTK
    renderWindowInteractor.Start() # Rozpoczęcie obsługi interakcji z oknem

    rclpy.shutdown() # Poprawne zamknięcie ROS2



if __name__ == '__main__':
    main()
