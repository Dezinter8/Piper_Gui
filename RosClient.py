from threading import Thread, Event
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage , LaserScan, JointState, Imu
from PyQt5.QtCore import QObject
import math

class RosClient(QObject):
    def __init__(self, visualizer, image_callback, enkoders, imu):
        super().__init__()
        self.visualizer = visualizer
        self.image_callback = image_callback
        self.enkoders = enkoders
        self.imu = imu 

        self.lidar_points = []

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
                lambda msg: self.enkoders.update_joints(msg.name, msg.position, msg.velocity), 
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
        # Wyczyszczenie listy punktów
        self.lidar_points = []

        for i, (range, intensity) in enumerate(zip(ranges, intensities)):
            if range == float('nan') or range == 0.0 or range == float('inf'):
                continue  # Pomijanie nieprawidłowych danych
            angle = angle_min + i * angle_increment
            x = (range * math.sin(angle)) * -1
            y = 0
            z = (range * math.cos(angle)) * -1
            self.lidar_points.append([x, y, z])

        # Wywołanie funkcji aktualizacji wizualizacji po aktualizacji punktów
        self.visualizer.update_visualization(self.lidar_points)

    def update_pivot(self, msg):
        angular_velocity = msg.angular_velocity
        # Obliczanie kierunku obrotu na podstawie wartości z żyroskopu
        rotation_direction = -1 if angular_velocity.z > 0 else 1

        for i, point in enumerate(self.lidar_points):
            rotation_angle = abs(angular_velocity.z)  
            transformation_distance = rotation_angle * 0.01  

            # Ustawianie odległości przekształcenia w zależności od kierunku obrotu
            if point[0] < 0:  # Jeśli punkt ma ujemną współrzędną x
                transformed_y = point[1] + (transformation_distance * rotation_direction)
            else:  # Jeśli punkt ma dodatnią współrzędną x
                transformed_y = point[1] - (transformation_distance * rotation_direction)

            self.lidar_points[i][1] = transformed_y

        # Wywołanie funkcji aktualizacji wizualizacji po przekształceniu punktów
        self.visualizer.update_visualization(self.lidar_points)
