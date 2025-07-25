from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QGroupBox, 
    QLineEdit, QLabel, QFrame, QGridLayout
)
from PyQt6.QtGui import QFont, QPalette, QColor
from PyQt6.QtCore import QProcess, Qt


class CamerasTab(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.apply_styles()

    def init_ui(self):
        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)

        # Left column - Camera controls
        left_column = QVBoxLayout()
        left_column.setSpacing(15)

        # IP Address Group
        ip_group = QGroupBox("Camera Configuration")
        ip_group.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        ip_layout = QVBoxLayout()
        ip_layout.setSpacing(10)

        ip_label = QLabel("Camera Station IP Address:")
        ip_label.setFont(QFont('Arial', 11))
        
        self.ip_input = QLineEdit()
        self.ip_input.setPlaceholderText("192.168.2.10")
        self.ip_input.setText("192.168.2.10")
        self.ip_input.setFont(QFont('Arial', 11))
        self.ip_input.setMinimumHeight(35)

        ip_layout.addWidget(ip_label)
        ip_layout.addWidget(self.ip_input)
        ip_group.setLayout(ip_layout)
        left_column.addWidget(ip_group)

        # Camera Streams Group
        streams_group = QGroupBox("Camera Streams")
        streams_group.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        streams_layout = QGridLayout()
        streams_layout.setVerticalSpacing(15)
        streams_layout.setHorizontalSpacing(15)

        self.cameras = [
            {"name": "📷 Camera 1 (Port 6123)", "port": 6123},
            {"name": "📷 Camera 2 (Port 7123)", "port": 7123},
            {"name": "📷 Camera 3 (Port 8123)", "port": 8123},
            {"name": "📷 Camera 4 (Port 9123)", "port": 9123},
        ]

        for i, cam in enumerate(self.cameras):
            button = QPushButton(cam["name"])
            button.setFont(QFont('Arial', 11))
            button.setMinimumHeight(50)
            button.setMinimumWidth(200)
            button.clicked.connect(lambda checked, p=cam["port"]: self.open_camera_stream(p))
            streams_layout.addWidget(button, i//2, i%2)

        streams_group.setLayout(streams_layout)
        left_column.addWidget(streams_group)
        left_column.addStretch()

        # Right column - Placeholder for future elements
        right_column = QFrame()
        right_column.setFrameShape(QFrame.Shape.StyledPanel)
        right_column.setLayout(QVBoxLayout())
        right_column.layout().addWidget(QLabel("Camera Preview Area"))
        right_column.layout().addStretch()
        right_column.setMinimumWidth(400)

        main_layout.addLayout(left_column)
        main_layout.addWidget(right_column)
        self.setLayout(main_layout)

    def apply_styles(self):
        self.setStyleSheet("""
            QWidget {
                background-color: #333;
                color: #ddd;
            }
            QGroupBox {
                font-weight: bold;
                border: 2px solid #555;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 15px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QPushButton {
                background-color: #444;
                border: 2px solid #555;
                border-radius: 6px;
                padding: 8px;
                min-width: 120px;
            }
            QPushButton:hover {
                background-color: #555;
                border: 2px solid #666;
            }
            QLineEdit {
                background-color: #3a3a3a;
                border: 1px solid #555;
                border-radius: 4px;
                padding: 8px;
                color: #ddd;
            }
            QLabel {
                padding: 5px;
            }
            QFrame {
                border-radius: 5px;
                background-color: #3a3a3a;
                border: 2px solid #555;
            }
        """)

    # def open_camera_stream(self, port):
    #     ip = self.ip_input.text() or "192.168.2.10"
        
    #     # Komenda GStreamer z dynamicznym IP i portem
    #     gst_command = f"gst-launch-1.0 -v udpsrc address={ip} port={port} ! application/x-rtp,encoding-name=H264 ! rtph264depay ! avdec_h264 ! queue ! autovideosink"

    #     # Otwieranie w nowym terminalu
    #     terminal_command = [
    #         "x-terminal-emulator", "-e", gst_command
    #     ]
        
    #     print(f"Otwieram terminal z komendą: {gst_command}")
        
    #     process = QProcess(self)
    #     process.startDetached(terminal_command[0], terminal_command[1:])

    def open_camera_stream(self, port):
        ip = self.ip_input.text() or "192.168.2.10"

        gst_command = f"gst-launch-1.0 -v udpsrc address={ip} port={port} ! application/x-rtp,encoding-name=H264 ! rtph264depay ! avdec_h264 ! queue ! autovideosink"

        terminal_command = [
            "gnome-terminal", "--", "bash", "-c", gst_command
        ]

        print(f"Otwieram terminal z komendą: {gst_command}")

        process = QProcess(self)
        process.startDetached(terminal_command[0], terminal_command[1:])
