from threading import Thread, Event
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage , LaserScan, JointState, Imu
from PyQt5.QtCore import QObject, pyqtSignal
import math
from datetime import datetime, timedelta

class RosClient(QObject):
    data_updated = pyqtSignal(dict)

    def __init__(self, main_window, visualizer, image_callback, enkoders, imu):
        super().__init__()
        self.main_window = main_window
        self.visualizer = visualizer
        self.image_callback = image_callback
        self.enkoders = enkoders
        self.imu = imu 

        self.lidar_points = []
        self.matplotlib_lidar_points = []
        
        self.wheelL = 0.0
        self.wheelR = 0.0
        self.wheelAvg = 0.0
        self.last_wheelL = 0.0
        self.last_wheelR = 0.0

        self.last_updated_time = datetime.now()
        self.vel_angle_z = 0.0
        self.vel_last_angle_z = 0.0

        self.acc_angle_x = 0.0
        self.acc_angle_y = 0.0
        self.acc_angle_z = 0.0

        self._is_running = Event()
        self._is_running.set()
        self.init_ros()

    def init_ros(self):
        self.thread = Thread(target=self.ros_thread_function, daemon=True)
        self.thread.start()

    def ros_thread_function(self):
        try:
            rclpy.init()
            self.node = Node("ros_client_node")
            
            #kamerka
            self.image_subscription = self.node.create_subscription(
                CompressedImage, '/image_raw/compressed', 
                self.image_callback, 
                10)
            
            #lidar
            self.scan_subscription = self.node.create_subscription(
                LaserScan, 'scan', 
                lambda msg: self.update_points(msg.ranges, msg.intensities, msg.angle_min, msg.angle_increment), 
                10)
            
            #enkodery
            self.subscription = self.node.create_subscription(
                JointState, '/joint_states', 
                lambda msg: self.update_joints(msg), 
                10)
            
            #akcelerometr
            self.subscription = self.node.create_subscription(
                Imu, '/imu_plugin/out', 
                lambda msg: self.update_pivot(msg), 
                100)
            
            while rclpy.ok() and self._is_running.is_set():
                rclpy.spin_once(self.node)
        except Exception as e:
            print(f"An error occurred in ROS thread: {e}")
        finally:
            if self.node:
                self.node.destroy_node()
            rclpy.shutdown()

    def shutdown(self):
        self._is_running.clear()
        if self.thread.is_alive():
            self.thread.join(timeout=2)  # Oczekuje do 5 sekund na zakończenie wątku
            if self.thread.is_alive():
                print("ROS thread did not terminate gracefully.")








    def update_points(self, ranges, intensities, angle_min, angle_increment):
        self.lidar_points = []
        color = []
        # Sprawdzenie, czy robot się porusza
        if self.wheelL == self.last_wheelL and self.wheelR == self.last_wheelR:
            # print("Robot się nie poruszył, pomijam aktualizację punktów lidaru")
            return  # Pominięcie aktualizacji punktów lidaru

        self.matplotlib_lidar_points = []

        for i, (range, intensity) in enumerate(zip(ranges, intensities)):
            if range == float('nan') or range == 0.0 or range == float('inf'):
                continue  # Pomijanie nieprawidłowych danych
            angle = angle_min + i * angle_increment
            x = (range * math.sin(angle)) * -1
            y = self.wheelAvg * 0.03      # Obliczenie y na podstawie liczby obrotów i promienia koła za pomocą wzory 2pi*r
            z = (range * math.cos(angle)) * -1

            self.lidar_points.append([x, y, z])
            # Dodanie punktu do listy punktów lidaru
            self.matplotlib_lidar_points.append([x, z])

            # Kolorowanie punktów na podstawie intensywności
            color.append(self.get_color_from_intensity(intensity))


        self.transform_points(color)


    def get_color_from_intensity(self, intensity):
        if math.isnan(intensity):  # Sprawdzenie czy intensywność jest NaN
            return [0, 0, 0]
        else:
            color_value = int(intensity)  # Skalowanie intensywności do wartości koloru (0-255)
            color = [color_value, 0, 0]  # Ustawienie RGB koloru
            return color

    def get_lidar_points(self):
        # print(self.lidar_points)
        return self.matplotlib_lidar_points

    def transform_points(self, color):
        transformed_points = []

        for point in self.lidar_points:
            # print(point)
            transformation_distance = point[0] * math.tan(math.radians(self.vel_angle_z))    # mierzenie długości przyprostokątnej a znając kąt alpha i długość przyprostokątnej b
            transformed_y = point[1] + transformation_distance
            transformed_points.append([point[0], transformed_y, point[2]])
            
        self.visualizer.update_visualization(transformed_points, color)

        # print(f'angle:  {self.vel_angle_z}      tan_angle:  {math.tan(math.radians(self.vel_angle_z))}')
        # print(f'wynik: {math.cos(math.radians(self.acc_angle_x)) * (self.wheelAvg * 0.03)}      acc_angle_x: {self.acc_angle_x}')


    def update_pivot(self, msg):
        linear_acceleration = msg.linear_acceleration
        angular_velocity = msg.angular_velocity

        data = {
            'linear_acc_x': linear_acceleration.x,
            'linear_acc_y': linear_acceleration.y,
            'linear_acc_z': linear_acceleration.z,
            'angular_vel_x': angular_velocity.x,
            'angular_vel_y': angular_velocity.y,
            'angular_vel_z': angular_velocity.z
        }
        
        
        if datetime.now() - self.last_updated_time > timedelta(seconds=1):
            self.last_updated_time = datetime.now()
        # Obliczanie kąta pochylenia
        dt = (datetime.now() - self.last_updated_time).total_seconds()

        # szacowanie obrotu robota na osi z
        self.vel_angle_z += (self.vel_last_angle_z + data['angular_vel_z'] * dt) * dt / 2
        self.vel_last_angle_z = data['angular_vel_z']

        # Kontrola zakresu kąta pochylenia
        if self.vel_angle_z > 180:
            self.vel_angle_z -= 360
        elif self.vel_angle_z < -180:
            self.vel_angle_z += 360


        # obliczanie pochylenia robota na osiach x y oraz z
        self.acc_angle_x = math.degrees(math.atan2(linear_acceleration.y, linear_acceleration.z))
        self.acc_angle_y = math.degrees(math.atan2(-linear_acceleration.x, linear_acceleration.z))
        self.acc_angle_z = math.degrees(math.atan2(linear_acceleration.x, linear_acceleration.y))


        self.data_updated.emit({
            'vel_angle_z': self.vel_angle_z,
            'acc_angle_x': self.acc_angle_x,
            'acc_angle_y': self.acc_angle_y,
            'acc_angle_z': self.acc_angle_z
        })

        # print(f"vel_z: {self.vel_angle_z} acc_x: {self.acc_angle_x} acc_y: {self.acc_angle_y} acc_z: {self.acc_angle_z}")









    # Enkodery
    def update_joints(self, msg):
        if not hasattr(self, 'start_wheelL') or not hasattr(self, 'start_wheelR'):
            # Ustawienie początkowych wartości tylko raz, gdy nie zostały jeszcze ustawione
            self.start_wheelL = msg.position[1]
            self.start_wheelR = msg.position[2]
        
        self.last_wheelL = self.wheelL
        self.last_wheelR = self.wheelR

        # Obliczenie różnicy pozycji aktualnej i startowej dla każdego koła
        wheelL_diff = round(msg.position[1] - self.start_wheelL, 3)
        wheelR_diff = round(msg.position[2] - self.start_wheelR, 3)

        self.wheelL = wheelL_diff
        self.wheelR = wheelR_diff
        self.wheelAvg = (self.wheelL + self.wheelR) / 2 

        # print(f'lewy: {self.wheelL} prawy: {self.wheelR} średnia: {self.wheelAvg}')
        # print(f'lewy: {self.last_wheelL} prawy: {self.last_wheelR}')

