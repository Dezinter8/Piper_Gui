import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage , Image, LaserScan
from PyQt5.QtCore import pyqtSignal, QObject
from cv_bridge import CvBridge
import subprocess

def check_lidar_connection():
    try:
        # Tworzenie tymczasowego węzła ROS 2
        node = rclpy.create_node('lidar_connection_checker')
        # Pobranie listy dostępnych tematów
        topic_names = node.get_topic_names_and_types()
        # Sprawdzenie czy istnieje temat związany z LiDARem (/scan)
        lidar_topic_exists = any('/scan' in topic for topic, _ in topic_names)
        
        # Pobranie listy tematów z polecenia 'ros2 topic list'
        result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
        topics_list = result.stdout.split('\n')

        # Sprawdzenie, czy 'scan' znajduje się na liście tematów
        lidar_topic_exists = '/scan' in topics_list
        
        return lidar_topic_exists
    
    except Exception as e:
        # Obsługa błędów podczas sprawdzania połączenia z LiDARem
        print("Błąd podczas sprawdzania połączenia z LiDARem:", e)
        return False

class RosClient(QObject):
    image_received = pyqtSignal(CompressedImage)

    def __init__(self):
        super().__init__()
        # Inicjalizacja ROS 2
        rclpy.init(args=None)
        # Utworzenie węzła ROS 2 dla klienta obrazu
        self.node = Node('image_viewer_node')
        # Inicjalizacja mostu OpenCV dla przekształceń obrazu
        self.bridge = CvBridge()
        # Utworzenie subskrypcji obrazu skompresowanego
        self.subscription = self.node.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, 10)

    def image_callback(self, msg):
        # Wywołanie sygnału informującego o otrzymaniu nowego obrazu
        self.image_received.emit(msg)

    def spin_once(self):
        # Wywołanie pojedynczej iteracji pętli ROS 2
        rclpy.spin_once(self.node, timeout_sec=0)

class LidarSubscriber(Node):
    ConectionStatus = None  # Przesunięcie tego atrybutu na poziom klasy

    def __init__(self, visualizer):
        super().__init__('lidar_subscriber')
        self.visualizer = visualizer
        
        # Sprawdzenie połączenia z LiDARem
        lidar_connected = check_lidar_connection()
        if lidar_connected:
            # Utworzenie subskrypcji dla danych z LiDARa
            self.subscription = self.create_subscription(
                LaserScan,
                'scan',
                self.listener_callback,
                10)
            self.get_logger().info('Subscriber initialized')
            LidarSubscriber.ConectionStatus = 'Utowrzono połączenie z topikiem scan - LiDAR' 
        else:
            self.get_logger().error('Failed to connect to LiDAR. Topic not found.')
            LidarSubscriber.ConectionStatus = 'Nie wykryto topiku scan - LiDAR'

    def listener_callback(self, msg):
        # Wywołanie metody wizualizatora do aktualizacji punktów na podstawie danych z LiDARa
        self.visualizer.update_points(msg.ranges, msg.angle_min, msg.angle_increment)
