from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QLabel, QGroupBox, QPushButton)
from PyQt6.QtGui import QFont
from PyQt6.QtCore import Qt, QProcess
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import os

class GPSTab(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.latitude = 0.0
        self.longitude = 0.0
        self.init_ui()
        self.init_ros_subscription()

    def init_ui(self):
        main_layout = QVBoxLayout()

        # Sekcja GPS
        gps_group = QGroupBox("GPS Data")
        gps_layout = QVBoxLayout()

        # Stworzenie większej czcionki
        font = QFont()
        font.setPointSize(14)

        self.latitude_label = QLabel("Latitude: Waiting for data...")
        self.latitude_label.setFont(font)
        self.longitude_label = QLabel("Longitude: Waiting for data...")
        self.longitude_label.setFont(font)

        gps_layout.addWidget(self.latitude_label)
        gps_layout.addWidget(self.longitude_label)
        gps_group.setLayout(gps_layout)

        # Duży przycisk do otwierania pliku gps.py
        open_gps_button = QPushButton("Open GPS Program")
        open_gps_button.setFont(font)
        open_gps_button.setFixedHeight(70)
        open_gps_button.setFixedWidth(300)
        open_gps_button.clicked.connect(self.open_gps_program)

        main_layout.addWidget(gps_group)
        main_layout.addWidget(open_gps_button)
        self.setLayout(main_layout)

    def init_ros_subscription(self):
        self.node.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)

    def gps_callback(self, msg: NavSatFix):
        """Obsługa nowych danych GPS"""
        self.latitude = msg.latitude
        self.longitude = msg.longitude

        self.latitude_label.setText(f"Latitude: {self.latitude:.6f}")
        self.longitude_label.setText(f"Longitude: {self.longitude:.6f}")

    def open_gps_program(self):
        """Otwiera plik gps.py jako osobny program w nowym okienku"""
        gps_program_path = os.path.join(os.path.dirname(__file__), 'GPS/gps.py')
        QProcess.startDetached('python', [gps_program_path])
