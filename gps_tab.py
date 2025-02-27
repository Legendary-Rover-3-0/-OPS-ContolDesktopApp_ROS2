from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QGroupBox
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import QUrl
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import folium
import os

class GPSTab(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.map_file = "gps_map.html"  # Dodaj to tutaj!
        self.init_ui()
        self.init_ros_subscription()

        
    def init_ui(self):
        main_layout = QVBoxLayout()

        gps_group = QGroupBox("GPS Data")
        gps_layout = QVBoxLayout()

        self.latitude_label = QLabel("Latitude: Waiting for data...")
        self.longitude_label = QLabel("Longitude: Waiting for data...")

        gps_layout.addWidget(self.latitude_label)
        gps_layout.addWidget(self.longitude_label)
        gps_group.setLayout(gps_layout)

        self.map_view = QWebEngineView()
        self.update_map(0, 0)  # Initial empty map
        
        main_layout.addWidget(gps_group)
        main_layout.addWidget(self.map_view)
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
        """)

    def init_ros_subscription(self):
        self.node.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)

    def gps_callback(self, msg: NavSatFix):
        self.latitude = msg.latitude
        self.longitude = msg.longitude

        self.latitude_label.setText(f"Latitude: {self.latitude:.6f}")
        self.longitude_label.setText(f"Longitude: {self.longitude:.6f}")
        
        self.update_map(self.latitude, self.longitude)

    def update_map(self, lat, lon):
        print(f"Updating map with coordinates: {lat}, {lon}")
        gps_map = folium.Map(location=[lat, lon], zoom_start=15)
        folium.Marker([lat, lon], popup=f"Lat: {lat}, Lon: {lon}").add_to(gps_map)

        try:
            gps_map.save(self.map_file)
            print(f"Map saved as {self.map_file}")
            
            # Poprawienie ścieżki URL dla QWebEngineView
            map_url = QUrl.fromLocalFile(os.path.abspath(self.map_file))
            self.map_view.setUrl(map_url)  # Teraz poprawny typ argumentu
        except Exception as e:
            print(f"Error saving map: {e}")
