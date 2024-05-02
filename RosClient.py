from threading import Thread, Event
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage , LaserScan, JointState, Imu
from PyQt5.QtCore import QObject

class RosClient(QObject):
    def __init__(self, visualizer, image_callback, enkoders, accelerometer):
        super().__init__()
        self.visualizer = visualizer
        self.image_callback = image_callback
        self.enkoders = enkoders
        self.accelerometer = accelerometer 

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
                lambda msg: self.visualizer.update_points(msg.ranges, msg.intensities, msg.angle_min, msg.angle_increment), 
                10)
            
            #enkodery
            self.subscription = self.node.create_subscription(
                JointState, '/joint_states', 
                lambda msg: self.enkoders.update_joints(msg.name, msg.position, msg.velocity), 
                10)
            
            #akcelerometr
            self.subscription = self.node.create_subscription(
                Imu, '/imu_plugin/out', 
                lambda msg: self.accelerometer.update_pivot(msg.orientation), 
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

