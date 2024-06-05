from threading import Thread, Event
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage , LaserScan, JointState
from geometry_msgs.msg import Point32, Quaternion
from std_msgs.msg import Int32
from PyQt5.QtCore import QObject, pyqtSignal, QTimer
import math
import numpy as np
import time

class RosClient(QObject):
    data_updated = pyqtSignal(dict)
    joints_updated = pyqtSignal(dict)

    def __init__(self, main_window, visualizer, image_callback, enkoders, imu, update_status_callback):
        super().__init__()
        self.main_window = main_window
        self.visualizer = visualizer
        self.image_callback = image_callback
        self.enkoders = enkoders
        self.imu = imu 
        self.update_status_callback = update_status_callback

        self.last_image_time = 0
        self.last_lidar_time = 0
        self.last_motors_time = 0
        self.last_imu_time = 0


        self.lidar_points = []
        self.matplotlib_lidar_points = []
        
        self.start_wheels = 0
        self.wheelL = 0.0
        self.wheelR = 0.0
        self.wheelAvg = 0.0
        self.last_wheelL = 0.0
        self.last_wheelR = 0.0
        self.robot_position = np.zeros(3)

        self.vel_angle_z = 0.0
        self.acc_angle_x = 0.0
        self.acc_angle_y = 0.0

        self.vel_angle_z_reset = 0.0  # dodana zmienna do przechowywania wartości resetu
        self.acc_angle_x_reset = 0.0
        self.acc_angle_y_reset = 0.0

        
        self._is_running = Event()
        self._is_running.set()
        self.init_ros()

        self.check_data_timer = QTimer()
        self.check_data_timer.timeout.connect(self.verify_data_reception)
        self.check_data_timer.start(1000)


    def init_ros(self):
        self.thread = Thread(target=self.ros_thread_function, daemon=True)
        self.thread.start()

    def ros_thread_function(self):
        try:
            rclpy.init()
            self.node = Node("ros_client_node")

            # Dodajemy publikator
            self.cmd_publisher = self.node.create_publisher(Point32, '/pico_subscription', 10)

            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=100
            )

            #kamerka
            self.image_subscription = self.node.create_subscription(
                CompressedImage, '/image_raw/compressed', 
                self.image_callback_wrapper, 
                10)
            
            #lidar
            self.scan_subscription = self.node.create_subscription(
                LaserScan, 'scan', 
                lambda msg: self.update_points(msg.ranges, msg.intensities, msg.angle_min, msg.angle_increment), 
                10)

            #enkodery
            self.subscription = self.node.create_subscription(
                Quaternion, '/enkoder_publisher', 
                lambda msg: self.update_joints(msg), 
                10)
            
            #akcelerometr
            self.subscription = self.node.create_subscription(
                Point32, '/imu_data', 
                lambda msg: self.update_pivot(msg), 
                qos_profile)

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
            self.thread.join(timeout=2)  # Oczekuje do 2 sekund na zakończenie wątku
            if self.thread.is_alive():
                print("ROS thread did not terminate gracefully.")


    def publish_command(self, x, y, z):
        msg = Point32()
        msg.x = x
        msg.y = y
        msg.z = z
        self.cmd_publisher.publish(msg)







    def update_points(self, ranges, intensities, angle_min, angle_increment):
        self.last_lidar_time = time.time()

        self.lidar_points = []
        color = []

        self.matplotlib_lidar_points = []

        for i, (range, intensity) in enumerate(zip(ranges, intensities)):
            if range == float('nan') or range == 0.0 or range == float('inf'):
                continue  # Pomijanie nieprawidłowych danych
            angle = angle_min + i * angle_increment
            x = (range * math.sin(angle)) * -1
            y = 0
            z = (range * math.cos(angle)) * -1

            self.lidar_points.append([x, y, z])
            # Dodanie punktu do listy punktów lidaru
            self.matplotlib_lidar_points.append([x, z])

            # Kolorowanie punktów na podstawie intensywności
            color.append(self.get_color_from_intensity(intensity))

        # Sprawdzenie, czy robot się porusza
        if self.wheelL == self.last_wheelL and self.wheelR == self.last_wheelR:
            # print("Robot się nie poruszył, pomijam aktualizację punktów lidaru")
            self.last_wheelL = self.wheelL
            self.last_wheelR = self.wheelR
            return  # Pominięcie aktualizacji punktów lidaru
        else:
            self.last_wheelL = self.wheelL
            self.last_wheelR = self.wheelR
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

        # Konwersja kątów rotacji na radiany
        alpha = np.radians(self.acc_angle_x)  # pitch
        beta = np.radians(self.acc_angle_y)  # roll
        gamma = np.radians(self.vel_angle_z)  # yaw

        # Macierze rotacji
        rx = np.array([[1, 0, 0],
                    [0, np.cos(alpha), -np.sin(alpha)],
                    [0, np.sin(alpha), np.cos(alpha)]])
        ry = np.array([[np.cos(beta), 0, np.sin(beta)],
                    [0, 1, 0],
                    [-np.sin(beta), 0, np.cos(beta)]])
        rz = np.array([[np.cos(gamma), -np.sin(gamma), 0],
                    [np.sin(gamma), np.cos(gamma), 0],
                    [0, 0, 1]])

        # Wykonanie transformacji dla każdego punktu
        for point in self.lidar_points:
            transformed_point = np.array(point)
            transformed_point = np.dot(rx, transformed_point)
            transformed_point = np.dot(ry, transformed_point)
            transformed_point = np.dot(rz, transformed_point)
            # Dodanie pozycji robota do punktu
            transformed_point += self.robot_position
            transformed_points.append(transformed_point)

        self.visualizer.update_visualization(transformed_points, color)





    def update_pivot(self, msg):
        self.last_imu_time = time.time()

        # self.acc_angle_x = msg.x
        # self.acc_angle_y = msg.y


        # Odejmowanie wartości resetu przed aktualizacją kąta
        adjusted_angle_x = msg.x - self.acc_angle_x_reset
        self.acc_angle_x = adjusted_angle_x

        adjusted_angle_y = msg.y - self.acc_angle_y_reset
        self.acc_angle_y = adjusted_angle_y

        adjusted_angle_z = msg.z - self.vel_angle_z_reset
        self.vel_angle_z = adjusted_angle_z

        # Emitowanie zaktualizowanych danych
        self.data_updated.emit({
            'vel_angle_z': self.vel_angle_z,
            'acc_angle_x': self.acc_angle_x,
            'acc_angle_y': self.acc_angle_y,
        })









    def update_joints(self, msg):
        self.last_motors_time = time.time()

        if self.start_wheels == 0:
            # Ustawienie początkowych wartości tylko raz, gdy nie zostały jeszcze ustawione
            self.start_wheelL = msg.z
            self.start_wheelR = msg.w

            self.start_wheels += 1

        self.last_wheelAvg = self.wheelAvg

        # Obliczenie różnicy pozycji aktualnej i startowej dla każdego koła
        wheelL_diff = round(msg.z - self.start_wheelL, 3)
        wheelR_diff = round(msg.w - self.start_wheelR, 3)

        self.wheelL = wheelL_diff
        self.wheelR = wheelR_diff
        self.wheelAvg = (self.wheelL + self.wheelR) / 2

        # Mierzenie skrętu na podstawie różnicy w ruchu koła
        WHEEL_RADIUS = 0.03

        # Aktualizacja pozycji na podstawie przemieszczenia i rotacji
        rx = np.array([[1, 0, 0],
                        [0, np.cos(np.radians(self.acc_angle_x)), -np.sin(np.radians(self.acc_angle_x))],
                          [0, np.sin(np.radians(self.acc_angle_x)), np.cos(np.radians(self.acc_angle_x))]])
        rz = np.array([[np.cos(np.radians(self.vel_angle_z)), -np.sin(np.radians(self.vel_angle_z)), 0],
                        [np.sin(np.radians(self.vel_angle_z)), np.cos(np.radians(self.vel_angle_z)), 0],
                          [0, 0, 1]])

        # Obliczanie przemieszczenia na podstawie różnicy położeń kół i rotacji
        displacement = np.array([0, (self.wheelAvg - self.last_wheelAvg) * WHEEL_RADIUS, 0])  # zakładam, że ruch wzdłuż osi y to przemieszczenie w przód/tył

        # Zastosowanie rotacji do przemieszczenia
        displacement = np.dot(rz, np.dot(rx, displacement))

        # Aktualizacja pozycji robota tylko gdy się poruszył
        if self.wheelAvg != self.last_wheelAvg:
            # Aktualizacja pozycji robota
            if not hasattr(self, 'robot_position'):
                self.robot_position = np.zeros(3)  # inicjalizacja pozycji robota

            self.robot_position += displacement

        # print(f'L: {self.wheelL}  R: {self.wheelR}')

        # Obliczenie prędkości liniowej dla obu kół
        left_wheel_speed = WHEEL_RADIUS * msg.x  # msg.x - prędkość lewego koła w rad/s
        right_wheel_speed = WHEEL_RADIUS * msg.y  # msg.y - prędkość prawego koła w rad/s

        # Średnia prędkość dla lepszego odzwierciedlenia prędkości robota
        average_speed = (left_wheel_speed + right_wheel_speed) / 2

        # Emitowanie sygnału z danymi, który zawiera także prędkości w m/s
        self.joints_updated.emit({
            'average_speed_mps': average_speed
        })



    def reset_visualization(self):
        self.start_wheels = 0
        self.wheelL = 0.0
        self.wheelR = 0.0
        self.wheelAvg = 0.0
        self.robot_position = np.zeros(3)

        # Zapisywanie aktualnego kąta jako wartość resetu
        self.vel_angle_z_reset += self.vel_angle_z
        self.acc_angle_x_reset += self.acc_angle_x
        self.acc_angle_y_reset += self.acc_angle_y



    def image_callback_wrapper(self, msg):
        self.last_image_time = time.time()

        self.image_callback(msg)  # Wywołanie oryginalnego callbacka




    def verify_data_reception(self):
        current_time = time.time()
        camera_status = "OK" if current_time - self.last_image_time < 2 else "Błąd"  # 2 sekund timeout
        lidar_status = "OK" if current_time - self.last_lidar_time < 2 else "Błąd"
        motors_status = "OK" if current_time - self.last_motors_time < 2 else "Błąd"
        imu_status = "OK" if current_time - self.last_imu_time < 2 else "Błąd"

        self.update_status_callback(camera_status, lidar_status, motors_status, imu_status)
