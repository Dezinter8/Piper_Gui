from threading import Thread, Event
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan
from PyQt5.QtCore import QObject

class RosClient(QObject):
    def __init__(self, visualizer, image_callback):
        super().__init__()
        self.visualizer = visualizer
        self.image_callback = image_callback
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
            self.image_subscription = self.node.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 10)
            self.scan_subscription = self.node.create_subscription(
                LaserScan, 'scan',
                lambda msg: self.visualizer.update_points(msg.ranges, msg.angle_min, msg.angle_increment),
                10
            )
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
            self.thread.join()  # Upewniamy się, że wątek zakończy działanie przed kontynuowaniem.

