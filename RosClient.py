from threading import Thread, Event
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage , LaserScan, JointState, Imu
from geometry_msgs.msg import Point32
from PyQt5.QtCore import QObject, pyqtSignal, QTimer
import math
from datetime import datetime
from filterpy.kalman import KalmanFilter
import numpy as np
import time

class RosClient(QObject):
    data_updated = pyqtSignal(dict)
    reset_complete = pyqtSignal()

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

        self.last_updated_time = datetime.now()
        self.vel_angle_z = 0.0

        self.acc_angle_x = 0.0
        self.acc_angle_y = 0.0

        self.pitch_gyro_weight = 0.98       # Waga dla pomiarów z żyroskopu
        self.pitch_accel_weight = 0.02      # Waga dla pomiarów z akcelerometru
        self.roll_gyro_weight = 0.98        # Waga dla pomiarów z żyroskopu
        self.roll_accel_weight = 0.02       # Waga dla pomiarów z akcelerometru

        # Inicjalizacja filtru Kalmana dla pomiaru kąta obrotu na osi z
        self.kf = KalmanFilter(dim_x=2, dim_z=1)
        self.kf.x = np.array([[0.], [0.]])          # początkowa wartość i prędkość kątowa
        self.kf.F = np.array([[1., 1.], [0., 1.]])  # macierz stanu
        self.kf.H = np.array([[1., 0.]])            # macierz obserwacji
        self.kf.P *= 1000.                          # macierz kowariancji
        self.kf.R = 0.01                            # macierz kowariancji pomiaru


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
            
            # Publikator do sterowania
            self.cmd_publisher = self.node.create_publisher(Point32, '/pico_subscription', 10)

            # Kamerka
            self.image_subscription = self.node.create_subscription(
                CompressedImage, '/image_raw/compressed', 
                self.image_callback_wrapper, 
                10)
            
            # Lidar
            self.scan_subscription = self.node.create_subscription(
                LaserScan, 'scan', 
                lambda msg: self.update_points(msg.ranges, msg.intensities, msg.angle_min, msg.angle_increment), 
                10)
            
            # Enkodery
            self.subscription = self.node.create_subscription(
                JointState, '/joint_states', 
                lambda msg: self.update_joints(msg), 
                10)
            
            # Akcelerometr
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
            self.thread.join(timeout=2)         # Oczekuje do 5 sekund na zakończenie wątku
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
                continue                                            # Pomijanie nieprawidłowych danych
            angle = angle_min + i * angle_increment
            x = (range * math.sin(angle)) * -1
            y = 0
            z = (range * math.cos(angle)) * -1

            self.lidar_points.append([x, y, z])
            self.matplotlib_lidar_points.append([x, z])             # Dodanie punktu do listy punktów lidaru
            color.append(self.get_color_from_intensity(intensity))  # Kolorowanie punktów na podstawie intensywności

        # Sprawdzenie, czy robot się porusza
        if self.wheelL == self.last_wheelL and self.wheelR == self.last_wheelR:
            self.last_wheelL = self.wheelL
            self.last_wheelR = self.wheelR
            return              # Pominięcie aktualizacji punktów lidaru
        else:
            self.last_wheelL = self.wheelL
            self.last_wheelR = self.wheelR
            self.transform_points(color)


    def get_color_from_intensity(self, intensity):
        if math.isnan(intensity):           # Sprawdzenie czy intensywność jest NaN
            return [0, 0, 0]
        else:
            color_value = int(intensity)    # Skalowanie intensywności do wartości koloru (0-255)
            color = [color_value, 0, 0]     # Ustawienie RGB koloru
            return color

    def get_lidar_points(self):
        return self.matplotlib_lidar_points





    def transform_points(self, color):
        transformed_points = []

        # Konwersja kątów rotacji na radiany
        alpha = np.radians(self.acc_angle_x)    # pitch
        beta = np.radians(self.acc_angle_y)     # roll
        gamma = np.radians(self.vel_angle_z)    # yaw

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
            transformed_point += self.robot_position            # Dodanie pozycji robota do punktu
            transformed_points.append(transformed_point)

        self.visualizer.update_visualization(transformed_points, color)





    def update_pivot(self, msg):
        self.last_imu_time = time.time()

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
        
        # Obliczanie czasu, który minął od ostatniej aktualizacji
        current_time = datetime.now()
        dt = (current_time - self.last_updated_time).total_seconds()
        self.last_updated_time = current_time

        if abs(data['angular_vel_z']) > 0.01:           # aktualizuj tylko gdy prędkość kątowa jest znacząca
                                                        # Przewidywanie stanu filtru Kalmana
            self.kf.F[0, 1] = dt                        # aktualizacja macierzy stanu
            self.kf.predict()
            self.kf.update(data['angular_vel_z'])       # Aktualizacja pomiaru kąta obrotu z żyroskopu
                                                        # Odczytanie oszacowanej wartości kąta obrotu z filtru Kalmana
            angle_change = self.kf.x[0, 0]              # Zmiana kąta obrotu od ostatniej aktualizacji
                                                        # Dodanie bieżącej zmiany kąta do łącznej wartości obrotu
            if abs(data['angular_vel_z']) > 0.01:       # Jeśli robot się obraca
                self.vel_angle_z += angle_change / 2    # yaw


        # Aktualizacja danych obrotu
                                                                                                                                    # Obliczanie kąta pitch na podstawie pomiarów z akcelerometru
        acc_pitch = math.degrees(math.atan2(linear_acceleration.y, math.sqrt(linear_acceleration.x**2 + linear_acceleration.z**2)))
                                                                                                                                    # Obliczanie zmiany kąta pitch na podstawie pomiarów z żyroskopu
        gyro_pitch_change = angular_velocity.z * 0.03                                                                               # Czas pomiędzy pomiarami to 0.03s
                                                                                                                                    # Połączenie obu pomiarów przy użyciu filtru komplementarnego
        self.acc_angle_x = self.pitch_gyro_weight * (self.acc_angle_x + gyro_pitch_change) + self.pitch_accel_weight * acc_pitch


        # Obliczanie kąta roll na podstawie pomiarów z akcelerometru
        acc_roll = math.degrees(math.atan2(-linear_acceleration.x, linear_acceleration.z))
                                                                                            # Obliczanie zmiany kąta roll na podstawie pomiarów z żyroskopu
        gyro_roll_change = angular_velocity.y * 0.03                                        # Czas pomiędzy pomiarami to 0.03s
                                                                                            # Połączenie obu pomiarów przy użyciu filtru komplementarnego
        self.acc_angle_y = self.roll_gyro_weight * (self.acc_angle_y + gyro_roll_change) + self.roll_accel_weight * acc_roll

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
            self.start_wheelL = msg.position[1]
            self.start_wheelR = msg.position[2]

            self.start_wheels += 1

        self.last_wheelAvg = self.wheelAvg

        # Obliczenie różnicy pozycji aktualnej i startowej dla każdego koła
        wheelL_diff = round(msg.position[1] - self.start_wheelL, 3)
        wheelR_diff = round(msg.position[2] - self.start_wheelR, 3)

        self.wheelL = wheelL_diff
        self.wheelR = wheelR_diff
        self.wheelAvg = (self.wheelL + self.wheelR) / 2

        WHEEL_RADIUS = 0.03

        # Aktualizacja pozycji na podstawie przemieszczenia i rotacji
        rx = np.array([[1, 0, 0],
                        [0, np.cos(np.radians(self.acc_angle_x)), -np.sin(np.radians(self.acc_angle_x))],
                          [0, np.sin(np.radians(self.acc_angle_x)), np.cos(np.radians(self.acc_angle_x))]])
        rz = np.array([[np.cos(np.radians(self.vel_angle_z)), -np.sin(np.radians(self.vel_angle_z)), 0],
                        [np.sin(np.radians(self.vel_angle_z)), np.cos(np.radians(self.vel_angle_z)), 0],
                          [0, 0, 1]])

        # Obliczanie przemieszczenia na podstawie różnicy położeń kół i rotacji
        displacement = np.array([0, (self.wheelAvg - self.last_wheelAvg) * WHEEL_RADIUS, 0])        # zakładamy, że ruch wzdłuż osi y to przemieszczenie w przód/tył
        displacement = np.dot(rz, np.dot(rx, displacement))                                         # Zastosowanie rotacji do przemieszczenia

        # Aktualizacja pozycji robota tylko gdy się poruszył
        if self.wheelAvg != self.last_wheelAvg:
            # Aktualizacja pozycji robota
            if not hasattr(self, 'robot_position'):
                self.robot_position = np.zeros(3)       # inicjalizacja pozycji robota

            self.robot_position += displacement



    def reset_visualization(self):
        self.start_wheels = 0
        self.wheelL = 0.0
        self.wheelR = 0.0
        self.wheelAvg = 0.0
        self.robot_position = np.zeros(3)

        self.vel_angle_z = 0.0
        self.acc_angle_x = 0.0
        self.acc_angle_y = 0.0


        self.reset_complete.emit()          # Emitowanie sygnału po zresetowaniu




    def image_callback_wrapper(self, msg):
        self.last_image_time = time.time()

        self.image_callback(msg)            # Wywołanie oryginalnego callbacka




    def verify_data_reception(self):
        current_time = time.time()
        camera_status = "OK" if current_time - self.last_image_time < 2 else "Błąd"     # 2 sekund timeout
        lidar_status = "OK" if current_time - self.last_lidar_time < 2 else "Błąd"
        motors_status = "OK" if current_time - self.last_motors_time < 2 else "Błąd"
        imu_status = "OK" if current_time - self.last_imu_time < 2 else "Błąd"

        self.update_status_callback(camera_status, lidar_status, motors_status, imu_status)
