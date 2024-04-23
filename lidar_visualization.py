import math
import time
import vtk
import rclpy
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
from RosClient import LidarSubscriber, JointStateSubscriber, ImuSubscriber

class LidarVisualizer:
    def __init__(self, renderer):
        # Initialization as per your existing code...
        self.points = vtk.vtkPoints()
        self.vertices = vtk.vtkCellArray()
        self.polyData = vtk.vtkPolyData()
        self.polyData.SetPoints(self.points)
        self.polyData.SetVerts(self.vertices)

        self.colors = vtk.vtkUnsignedCharArray()
        self.colors.SetNumberOfComponents(3)
        self.colors.SetName("Colors")
        self.polyData.GetPointData().SetScalars(self.colors)

        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInputData(self.polyData)
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)
        self.actor.GetProperty().SetPointSize(5)
        self.actor.GetProperty().SetColor(1.0, 0.0, 0.0)  # Red points

        self.z_offset = 0
        self.last_update_time = time.time()
        self.added_points = {}

    
    def update_joints(self, name, position, velocity):

        print(name,position,velocity)

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
    rclpy.init(args=args)

    renderer = vtk.vtkRenderer()
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.SetSize(600, 500)
    renderWindow.SetWindowName("Wizualizacja Liadru")
    renderWindow.AddRenderer(renderer)
    
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)

    visualizer = LidarVisualizer(renderer)
    lidar_subscriber = LidarSubscriber(visualizer)
    
    enkoders_visualizer = LidarVisualizer(renderer)
    joint_state_subscriber = JointStateSubscriber(enkoders_visualizer)
    
    imu_subscriber = ImuSubscriber()

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

    # Use a MultiThreadedExecutor to handle the nodes
    executor = MultiThreadedExecutor()
    executor.add_node(lidar_subscriber)
    executor.add_node(joint_state_subscriber)
    executor.add_node(imu_subscriber)

    # Spin in a separate thread
    rclpy_thread = Thread(target=executor.spin, daemon=True)
    rclpy_thread.start()

    renderWindow.Render()
    renderWindowInteractor.Start()

    rclpy.shutdown()

if __name__ == '__main__':
    main()