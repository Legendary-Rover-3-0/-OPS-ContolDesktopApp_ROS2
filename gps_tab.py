from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QLabel, QGroupBox, QPushButton, 
                             QHBoxLayout, QLineEdit, QListWidget, QListWidgetItem, 
                             QAbstractItemView)
from PyQt6.QtGui import QFont, QIcon
from PyQt6.QtCore import Qt, QProcess, QTimer, QThread, pyqtSignal
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
import os
import sqlite3
from datetime import datetime
import time

class DatabaseWatcher(QThread):
    update_needed = pyqtSignal()



    def __init__(self, db_path):
        super().__init__()
        self.db_path = db_path
        self.running = True
        self.last_count = 0

    def run(self):
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        while self.running:
            try:
                cursor.execute("SELECT COUNT(*) FROM targets")
                current_count = cursor.fetchone()[0]
                
                if current_count != self.last_count:
                    self.last_count = current_count
                    self.update_needed.emit()
                
                time.sleep(2)  # Sprawdzaj co 2 sekundy
            except sqlite3.Error as e:
                print(f"Database error: {e}")
                time.sleep(2)
        
        conn.close()

    def stop(self):
        self.running = False
        self.wait()

class GPSTab(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.latitude = 0.0
        self.longitude = 0.0
        self.init_db()
        self.init_ui()
        self.setup_database_watcher()

        self.node.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.target_publisher = self.node.create_publisher(Float64MultiArray, '/gps/targets', 10)

    def init_db(self):
        """Inicjalizacja bazy danych SQLite"""
        self.db_connection = sqlite3.connect('GPS/gps_targets.db', check_same_thread=False)
        self.cursor = self.db_connection.cursor()
        
        # Tworzenie tabeli jeśli nie istnieje
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS targets (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                latitude REAL NOT NULL,
                longitude REAL NOT NULL,
                timestamp TEXT NOT NULL
            )
        ''')
        self.db_connection.commit()

    def setup_database_watcher(self):
        """Uruchamia wątek monitorujący zmiany w bazie danych"""
        self.watcher = DatabaseWatcher('GPS/gps_targets.db')
        self.watcher.update_needed.connect(self.load_targets_from_db)
        self.watcher.start()

    def init_ui(self):
        main_layout = QHBoxLayout()

        # Lewa strona - obecne dane GPS
        left_layout = QVBoxLayout()
        
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

        left_layout.addWidget(gps_group)
        left_layout.addWidget(open_gps_button)
        left_layout.addStretch()

        # Prawa strona - zarządzanie celami
        right_layout = QVBoxLayout()
        
        # Sekcja dodawania nowego celu
        add_target_group = QGroupBox("Add Target")
        add_target_layout = QVBoxLayout()
        
        # Pola do wprowadzania współrzędnych
        self.lat_input = QLineEdit()
        self.lat_input.setPlaceholderText("Enter latitude")
        self.lon_input = QLineEdit()
        self.lon_input.setPlaceholderText("Enter longitude")
        
        # Przycisk do dodawania ręcznie wprowadzonych współrzędnych
        add_manual_btn = QPushButton("Add Target Coordinates")
        add_manual_btn.clicked.connect(self.add_manual_coordinates)
        add_manual_btn.setFont(font)
        add_manual_btn.setFixedHeight(50)
        
        add_target_layout.addWidget(QLabel("Latitude:"))
        add_target_layout.addWidget(self.lat_input)
        add_target_layout.addWidget(QLabel("Longitude:"))
        add_target_layout.addWidget(self.lon_input)
        add_target_layout.addWidget(add_manual_btn)
        add_target_group.setLayout(add_target_layout)
        
        # Lista celów
        targets_group = QGroupBox("Target List")
        targets_layout = QVBoxLayout()
        
        self.targets_list = QListWidget()
        self.targets_list.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.targets_list.itemDoubleClicked.connect(self.on_item_double_clicked)
        
        targets_layout.addWidget(self.targets_list)
        targets_group.setLayout(targets_layout)
        
        right_layout.addWidget(add_target_group)
        right_layout.addWidget(targets_group)

        self.send_targets_button = QPushButton("Send target list")
        self.send_targets_button.clicked.connect(self.send_targets)
        self.send_targets_button.setFont(font)
        self.send_targets_button.setFixedHeight(50)

        right_layout.addWidget(self.send_targets_button)
        
        # Łączenie głównych layoutów
        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)
        self.setLayout(main_layout)
        
        # Wczytanie istniejących celów z bazy danych
        self.load_targets_from_db()
        

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

    def add_manual_coordinates(self):
        """Dodaje ręcznie wprowadzone współrzędne jako cel"""
        try:
            lat = float(self.lat_input.text())
            lon = float(self.lon_input.text())
            self.add_target_to_db(lat, lon)
            self.load_targets_from_db()
            self.lat_input.clear()
            self.lon_input.clear()
        except ValueError:
            self.node.get_logger().warn("Invalid coordinates format")

    def add_target_to_db(self, latitude, longitude):
        """Dodaje cel do bazy danych"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.cursor.execute(
            "INSERT INTO targets (latitude, longitude, timestamp) VALUES (?, ?, ?)",
            (latitude, longitude, timestamp)
        )
        self.db_connection.commit()

    def load_targets_from_db(self):
        """Wczytuje cele z bazy danych i wyświetla je na liście"""
        self.targets_list.clear()
        self.cursor.execute("SELECT id, latitude, longitude, timestamp FROM targets ORDER BY timestamp DESC")
        targets = self.cursor.fetchall()
        
        for target in targets:
            item = QListWidgetItem()
            item.setData(Qt.ItemDataRole.UserRole, target[0])  # Przechowujemy ID w danych użytkownika
            
            widget = QWidget()
            layout = QHBoxLayout()
            
            # Etykieta z informacjami o celu
            label = QLabel(f"Lat: {target[1]}, Lon: {target[2]}\n{target[3]}")
            
            # Przycisk do usuwania
            delete_btn = QPushButton()
            delete_btn.setIcon(QIcon.fromTheme("edit-delete"))
            delete_btn.setFixedSize(30, 30)
            delete_btn.clicked.connect(lambda _, item=item: self.delete_target(item))
            
            layout.addWidget(label)
            layout.addWidget(delete_btn)
            layout.setContentsMargins(5, 5, 5, 5)
            
            widget.setLayout(layout)
            
            item.setSizeHint(widget.sizeHint())
            self.targets_list.addItem(item)
            self.targets_list.setItemWidget(item, widget)

    def delete_target(self, item):
        """Usuwa wybrany cel z bazy danych i listy"""
        target_id = item.data(Qt.ItemDataRole.UserRole)
        self.cursor.execute("DELETE FROM targets WHERE id = ?", (target_id,))
        self.db_connection.commit()
        self.load_targets_from_db()

    def on_item_double_clicked(self, item):
        """Obsługa podwójnego kliknięcia na element listy"""
        target_id = item.data(Qt.ItemDataRole.UserRole)
        self.cursor.execute("SELECT latitude, longitude FROM targets WHERE id = ?", (target_id,))
        lat, lon = self.cursor.fetchone()
        self.lat_input.setText(str(lat))
        self.lon_input.setText(str(lon))

    def send_targets(self):
        """Wysyła listę targetów jako Float32MultiArray w formacie [lat1, lon1, lat2, lon2, ...]"""
        self.cursor.execute("SELECT latitude, longitude FROM targets")
        rows = self.cursor.fetchall()

        data = []
        for lat, lon in rows:
            data.append(float(lat))
            data.append(float(lon))

        msg = Float64MultiArray()
        msg.data = data

        self.target_publisher.publish(msg)
        #self.node.get_logger().info(f"Wysłano {len(rows)} targetów do /gps/targets jako Float32MultiArray")


    def closeEvent(self, event):
        """Zamykanie zasobów przy zamykaniu aplikacji"""
        self.watcher.stop()
        self.db_connection.close()
        super().closeEvent(event)
