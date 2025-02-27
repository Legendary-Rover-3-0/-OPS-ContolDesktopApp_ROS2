from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QGroupBox, QPushButton, QHBoxLayout
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import Qt
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import requests
from io import BytesIO
import os

class GPSTab(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.zoom_level = 15  # Domyślny zoom mapy
        self.init_ui()
        self.init_ros_subscription()

    def init_ui(self):
        main_layout = QVBoxLayout()

        # Sekcja GPS
        gps_group = QGroupBox("GPS Data")
        gps_layout = QVBoxLayout()

        self.latitude_label = QLabel("Latitude: Waiting for data...")
        self.longitude_label = QLabel("Longitude: Waiting for data...")

        gps_layout.addWidget(self.latitude_label)
        gps_layout.addWidget(self.longitude_label)
        gps_group.setLayout(gps_layout)

        # Sekcja mapy
        self.map_label = QLabel()
        self.map_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.load_default_map()
        
        # Przyciski do zoomowania
        zoom_layout = QHBoxLayout()
        self.zoom_in_button = QPushButton("+")
        self.zoom_out_button = QPushButton("-")
        self.zoom_in_button.clicked.connect(self.zoom_in)
        self.zoom_out_button.clicked.connect(self.zoom_out)
        
        zoom_layout.addWidget(self.zoom_out_button)
        zoom_layout.addWidget(self.zoom_in_button)

        main_layout.addWidget(gps_group)
        main_layout.addWidget(self.map_label)
        main_layout.addLayout(zoom_layout)
        self.setLayout(main_layout)

        self.setStyleSheet("""
            QLabel {
                font-size: 12px;
                color: #ddd;
            }
            QGroupBox {
                font-size: 14px;
                font-weight: bold;
                margin-top: 10px;
                background-color: #444;
                color: #ddd;
            }
            QWidget {
                background-color: #333;
            }
            QPushButton {
                font-size: 16px;
                background-color: #555;
                color: white;
                border-radius: 5px;
                padding: 5px;
            }
            QPushButton:hover {
                background-color: #777;
            }
        """)

    def init_ros_subscription(self):
        self.node.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)

    def gps_callback(self, msg: NavSatFix):
        """Obsługa nowych danych GPS"""
        self.latitude = msg.latitude
        self.longitude = msg.longitude

        self.latitude_label.setText(f"Latitude: {self.latitude:.6f}")
        self.longitude_label.setText(f"Longitude: {self.longitude:.6f}")
        
        self.update_map(self.latitude, self.longitude)

    def update_map(self, lat, lon):
        """Aktualizacja mapy z nowymi współrzędnymi"""
        map_url = (f"https://maps.googleapis.com/maps/api/staticmap?center={lat},{lon}" 
                   f"&zoom={self.zoom_level}&size=600x400&maptype=roadmap&markers=color:red%7C{lat},{lon}&key=TWÓJ_KLUCZ_API")
        try:
            response = requests.get(map_url)
            if response.status_code == 200:
                pixmap = QPixmap()
                pixmap.loadFromData(response.content)
                self.map_label.setPixmap(pixmap)
            else:
                print(f"Failed to fetch map: {response.status_code}")
        except Exception as e:
            print(f"Error fetching map: {e}")

    def zoom_in(self):
        if self.zoom_level < 20:
            self.zoom_level += 1
            self.update_map(self.latitude, self.longitude)

    def zoom_out(self):
        if self.zoom_level > 1:
            self.zoom_level -= 1
            self.update_map(self.latitude, self.longitude)

    def load_default_map(self):
        """Ładuje domyślną mapę z pliku"""
        map_path = os.path.join(os.path.dirname(__file__), "default_map.png")
        if os.path.exists(map_path):
            pixmap = QPixmap(map_path)
            self.map_label.setPixmap(pixmap)
        else:
            self.map_label.setText("Brak domyślnej mapy.")
