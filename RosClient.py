import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage , Image, LaserScan
from PyQt5.QtCore import pyqtSignal, QObject
from cv_bridge import CvBridge
import subprocess
from threading import Thread

class RosClient(QObject):
    def __init__(self, visualizer, image_callback):
        super().__init__()
        self.visualizer = visualizer
        self.image_callback = image_callback
        self.init_ros()

    def init_ros(self):
        self.thread = Thread(target=self.ros_thread_function)
        self.thread.start()

    def ros_thread_function(self):
        rclpy.init()
        self.node = Node("ros_client_node")

        # Subskrypcja obrazu
        self.image_subscription = self.node.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 10)
        
        # Subskrypcja skanu lidar
        self.scan_subscription = self.node.create_subscription(LaserScan, 'scan', self.visualizer.listener_callback, 10)
        
        rclpy.spin(self.node)

    def shutdown(self):
        rclpy.shutdown()



class ImageSubscriber(QObject):
    image_received = pyqtSignal(CompressedImage)

    def __init__(self):
        super().__init__()
        rclpy.init(args=None)
        self.node = Node('image_viewer_node')
        self.bridge = CvBridge()
        self.subscription = self.node.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 10)

    def image_callback(self, msg):
        self.image_received.emit(msg)

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0)

class LidarSubscriber(Node):
    def __init__(self, visualizer):
        super().__init__('lidar_subscriber')
        self.visualizer = visualizer
        
        self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)

        self.get_logger().info('Subscriber initialized')
        LidarSubscriber.ConectionStatus = 'Utowrzono połączenie z topikiem scan - LiDAR' 

    def listener_callback(self, msg):
        self.visualizer.update_points(msg.ranges, msg.angle_min, msg.angle_increment)
