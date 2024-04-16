import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from PyQt5.QtCore import pyqtSignal, QObject
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan

class RosClient(QObject):
    image_received = pyqtSignal(CompressedImage)

    def __init__(self):
        super().__init__()
        rclpy.init(args=None)
        self.node = Node('image_viewer_node')
        self.bridge = CvBridge()
        self.subscription = self.node.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, 10)

    def image_callback(self, msg):
        self.image_received.emit(msg)

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0)

class LidarSubscriber(Node):
    def __init__(self, visualizer):
        # Inicjalizacja węzła ROS2 z nazwą 'lidar_subscriber'.
        super().__init__('lidar_subscriber')
        self.visualizer = visualizer  # Przechowanie referencji do wizualizera.
        
        # Subskrypcja do tematu 'scan' z wiadomościami LaserScan.
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)  # Ustawienie rozmiaru kolejki na 10.
        self.get_logger().info('Subscriber initialized')

    def listener_callback(self, msg):
        # Callback wywoływany przy otrzymaniu nowej wiadomości.
        # Aktualizacja wizualizacji punktów na podstawie danych z wiadomości.
        self.visualizer.update_points(msg.ranges, msg.angle_min, msg.angle_increment)
