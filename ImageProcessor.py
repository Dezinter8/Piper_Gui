from PyQt5.QtGui import QImage, QPixmap
import cv2
from datetime import datetime

class ImageProcessor:
    def __init__(self):
        self.recording = False
        self.video_writer = None

    def convert_cv_to_pixmap(self, cv_img):
        height, width, channel = cv_img.shape
        bytes_per_line = channel * width
        qt_image = QImage(cv_img.data, width, height, bytes_per_line, QImage.Format_BGR888)
        return QPixmap.fromImage(qt_image)

    def start_recording(self):
        now = datetime.now()
        filename = now.strftime("%Y-%m-%d_%H-%M-%S_Kamera.avi")
        self.video_writer = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*'MJPG'), 20, (640, 480))
        self.recording = True

    def stop_recording(self):
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
        self.recording = False

    def write_frame(self, cv_img):
        if self.recording and self.video_writer:
            self.video_writer.write(cv_img)

