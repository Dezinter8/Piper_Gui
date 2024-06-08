import cv2
import os
from PyQt5.QtGui import QImage, QPixmap
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
        try:
            now = datetime.now()
            output_directory = os.path.expanduser("~/piper_output")     # uzyskanie ścieżka do katalogu domowego użytkownika
            self.filename = os.path.join(output_directory, now.strftime("%Y-%m-%d_%H-%M-%S_Kamera.avi"))
            os.makedirs(output_directory, exist_ok=True)
            self.video_writer = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*'MJPG'), 15, (640, 480))
            self.recording = True
        except Exception as e:
            print(f"Error: {e}")


    def stop_recording(self):
        if self.video_writer:
            try:
                self.video_writer.release()
                filename = self.filename
                self.video_writer = None
                self.recording = False
                return True, filename
            except Exception as e:
                self.video_writer = None
                self.recording = False
                return False, str(e)        # Zwracanie False i komunikatu o błędzie
        self.recording = False
        return None, "Nagrywanie nie było aktywne."


    def write_frame(self, cv_img):
        if self.recording and self.video_writer:
            self.video_writer.write(cv_img)

