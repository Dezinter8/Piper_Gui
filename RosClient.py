import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage , Image, LaserScan, JointState, Imu
from PyQt5.QtCore import pyqtSignal, QObject
from cv_bridge import CvBridge
import subprocess

def check_connection(name):
    try:
        # Tworzenie tymczasowego węzła ROS 2
        node = rclpy.create_node('connection_checker')
        # Pobranie listy dostępnych tematów
        topic_names = node.get_topic_names_and_types()
        # Sprawdzenie czy istnieje temat związany z LiDARem (/scan)
        lidar_topic_exists = any('/scan' in topic for topic, _ in topic_names)
        
        # Pobranie listy tematów z polecenia 'ros2 topic list'
        result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
        topics_list = result.stdout.split('\n')

        # Sprawdzenie, czy topic znajduje się na liście tematów
        if name == 'lidar':
            lidar_topic_exists = '/scan' in topics_list
            return lidar_topic_exists
        elif name =='enkoder':
            joint_state_topic_exists = '/joint_states' in topics_list
            return joint_state_topic_exists
        elif name =='akcelerometr':
            imu_topic_exists = '/imu_plugin/out' in topics_list
            return imu_topic_exists
        
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

#   LIDAR
class LidarSubscriber(Node):
    ConectionStatus = None  # Atrybut klasy przechowujący informacje o stanie połączenia 

    def __init__(self, visualizer):
        super().__init__('lidar_subscriber')
        self.visualizer = visualizer
        
        # Sprawdzenie połączenia z LiDARem
        lidar_connected = check_connection('lidar')
        if lidar_connected:
            # Utworzenie subskrypcji dla danych z LiDARa
            self.subscription = self.create_subscription(
                LaserScan,
                'scan',
                self.listener_callback,
                10)
            self.get_logger().info('Utowrzono połączenie z topikiem scan - LiDAR')
        else:
            self.get_logger().error('Nie wykryto topiku scan - LiDAR')

    def listener_callback(self, msg):
        # Wywołanie metody wizualizatora do aktualizacji punktów na podstawie danych z LiDARa
        self.visualizer.update_points(msg.ranges, msg.angle_min, msg.angle_increment)
    
    
#   ENKODERY 
class JointStateSubscriber(Node):
    ConnectionStatus = None  # Atrybut klasy przechowujący informacje o stanie połączenia

    def __init__(self, enkoders):
        super().__init__('joint_state_subscriber')
        self.enkoders = enkoders
        
        # Sprawdzanie, czy topic '/joint_states' jest dostępny
        enkoder_connected = check_connection('enkoder')
        if enkoder_connected:
            # Utworzenie subskrypcji dla danych z '/joint_states'
            self.subscription = self.create_subscription(
                JointState,
                '/joint_states',
                self.listener_callback,
                10)
            self.get_logger().info('Utworzono połączenie z topikiem joint_states - enkodery')
        else:
            self.get_logger().error('Nie wykryto topiku joint_states - enkodery')

    def listener_callback(self, msg):
        # Wywołanie metody enkoders do aktualizacji informacji na podstawie danych z /joint_state
        self.enkoders.update_joints(msg.name, msg.position, msg.velocity)


#   AKCELEROMETR
class ImuSubscriber(Node):
    ConnectionStatus = None  # Atrybut klasy przechowujący informacje o stanie połączenia
    
    def __init__(self, accelerometer):
        super().__init__('imu_subscriber')
        self.accelerometer = accelerometer        
                
        # Sprawdzanie, czy topic '/imu_plugin/out' jest dostępny
        akcelerometr_connected = check_connection('akcelerometr')
        if akcelerometr_connected:
            # Utworzenie subskrypcji dla danych z '/imu_plugin/out'
            self.subscription = self.create_subscription(
                Imu,
                '/imu_plugin/out',
                self.listener_callback,
                10)
            self.get_logger().info('Utworzono połączenie z topikiem imu_plugin/out - akcelerometr')
        else:
            self.get_logger().error('Nie wykryto topiku imu_plugin/out - akcelerometr')

    def listener_callback(self, msg):
        # Wywołanie metody accelerometers do aktualizacji informacji na podstawie danych z /imu_plugin/out
        self.accelerometer.update_pivot(msg.orientation)