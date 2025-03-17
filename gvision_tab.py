from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QPushButton, QGroupBox, QLineEdit, QLabel
)
from PyQt6.QtGui import QFont
from PyQt6.QtCore import QProcess


class CamerasTab(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout()

        # Pole do wpisania IP
        ip_label = QLabel("Wprowadź adres IP stacji (domyślnie 192.168.2.10):")
        self.ip_input = QLineEdit()
        self.ip_input.setPlaceholderText("192.168.2.10")
        self.ip_input.setText("192.168.2.10")  # domyślna wartość

        main_layout.addWidget(ip_label)
        main_layout.addWidget(self.ip_input)

        cameras_group = QGroupBox("Camera Streams")
        group_layout = QVBoxLayout()

        font = QFont()
        font.setPointSize(14)

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
        ip = self.ip_input.text() or "192.168.2.98"
        
        # Komenda GStreamer z dynamicznym IP i portem
        gst_command = f"gst-launch-1.0 -v udpsrc address={ip} port={port} ! application/x-rtp,encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! queue ! autovideosink"

        # Otwieranie w nowym terminalu
        terminal_command = [
            "x-terminal-emulator", "-e", gst_command
        ]
        
        # Jeśli wolisz użyć np. gnome-terminal, zamień na:
        # terminal_command = ["gnome-terminal", "--", "bash", "-c", f"{gst_command}; exec bash"]

        print(f"Otwieram terminal z komendą: {gst_command}")
        
        process = QProcess(self)
        process.startDetached(terminal_command[0], terminal_command[1:])


