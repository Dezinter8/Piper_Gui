import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from PyQt5.QtCore import pyqtSignal, QObject
from cv_bridge import CvBridge
import subprocess

def check_lidar_connection():
    try:
        node = rclpy.create_node('lidar_connection_checker')
        topic_names = node.get_topic_names_and_types()
        lidar_topic_exists = any('/scan' in topic for topic, _ in topic_names)
        
        # Pobranie listy tematów z polecenia 'ros2 topic list'
        result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
        topics_list = result.stdout.split('\n')

        # Sprawdzenie, czy 'scan' znajduje się na liście tematów
        lidar_topic_exists = '/scan' in topics_list
        
        return lidar_topic_exists
    
    except Exception as e:
        print("Błąd podczas sprawdzania połączenia z LiDARem:", e)
        return False

class RosClient(QObject):
    image_received = pyqtSignal(Image)

    def __init__(self):
        super().__init__()
        rclpy.init(args=None)
        self.node = Node('image_viewer_node')
        self.bridge = CvBridge()
        self.subscription = self.node.create_subscription(
            Image, '/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        self.image_received.emit(msg)

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0)

class LidarSubscriber(Node):
    ConectionStatus = None  # Przesunięcie tego atrybutu na poziom klasy

    def __init__(self, visualizer):
        super().__init__('lidar_subscriber')
        self.visualizer = visualizer
        
        lidar_connected = check_lidar_connection()
        if lidar_connected:
            self.subscription = self.create_subscription(
                LaserScan,
                'scan',
                self.listener_callback,
                10)
            self.get_logger().info('Subscriber initialized')
            LidarSubscriber.ConectionStatus = 'LiDAR OK'  # Ustawienie atrybutu klasy
        else:
            self.get_logger().error('Failed to connect to LiDAR. Topic not found.')
            LidarSubscriber.ConectionStatus = 'LiDAR ERROR'  # Ustawienie atrybutu klasy

    def listener_callback(self, msg):
        self.visualizer.update_points(msg.ranges, msg.angle_min, msg.angle_increment)

