from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QGroupBox)
from PyQt6.QtGui import QFont
from PyQt6.QtCore import QProcess

class CamerasTab(QWidget):
    def __init__(self):
        super().__init__()
        self.processes = []  # lista do przechowywania procesów
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout()

        cameras_group = QGroupBox("Camera Streams")
        group_layout = QVBoxLayout()

        font = QFont()
        font.setPointSize(14)

        # Definicja kamer z portami i adresami
        self.cameras = [
            {"name": "Camera 1 (Port 6123)", "port": 6123},
            {"name": "Camera 2 (Port 7123)", "port": 7123},
            {"name": "Camera 3 (Port 8123)", "port": 8123},
            {"name": "Camera 4 (Port 9123)", "port": 9123},
        ]

        for cam in self.cameras:
            button = QPushButton(cam["name"])
            button.setFont(font)
            button.setFixedHeight(70)
            button.setFixedWidth(350)
            button.clicked.connect(lambda checked, p=cam["port"]: self.open_camera_stream(p))
            group_layout.addWidget(button)

        cameras_group.setLayout(group_layout)
        main_layout.addWidget(cameras_group)
        self.setLayout(main_layout)

    def open_camera_stream(self, port):
        """Uruchamia proces GStreamer dla konkretnego portu"""
        command = [
            "gst-launch-1.0",
            "-v",
            f"udpsrc", f"address=192.168.2.2", f"port={port}",
            "!",
            "application/x-rtp,encoding-name=H264",
            "!",
            "rtph264depay",
            "!",
            "h264parse",
            "!",
            "avdec_h264",
            "!",
            "queue",
            "!",
            "autovideosink"
        ]
        process = QProcess(self)
        process.startDetached(" ".join(command))
        self.processes.append(process)  # opcjonalnie, aby trzymać referencję
