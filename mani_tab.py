import threading
import pygame
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QGroupBox
from PyQt6.QtCore import Qt
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String  # Dodano String dla danych RFID

class ManipulatorTab(QWidget):
    def __init__(self, node: Node, gamepads):
        super().__init__()
        self.node = node
        self.gamepads = gamepads
        self.selected_gamepad = None
        self.running = False

        # Inicjalizacja wartości dla 6 stopni swobody
        self.current_values = [0.0] * 6  # Zmiana na wartości zmiennoprzecinkowe
        self.sensitivity = 50.0  # Domyślny krok ruchu, regulowany suwakiem

        # Inicjalizacja danych RFID
        self.rfid_data = "Brak danych"

        self.init_ui()
        self.init_ros_publishers()
        self.init_ros_subscribers()

    def init_ui(self):
        main_layout = QHBoxLayout()  # Główny układ poziomy

        # Kolumna dla manipulatora
        self.mani_group = QGroupBox("Manipulator Control")
        mani_layout = QVBoxLayout()

        # Etykiety do wyświetlania aktualnego ruchu
        self.movement_labels = [QLabel(f"Stopień {i+1}: 0.0") for i in range(6)]
        for label in self.movement_labels:
            mani_layout.addWidget(label)

        # Suwak do zmiany czułości ruchu
        self.sensitivity_slider = QSlider(Qt.Orientation.Horizontal)
        self.sensitivity_slider.setMinimum(1)
        self.sensitivity_slider.setMaximum(100)
        self.sensitivity_slider.setValue(int(self.sensitivity))  # Ustawienie wartości początkowej
        self.sensitivity_slider.valueChanged.connect(self.update_sensitivity)

        mani_layout.addWidget(QLabel("Czułość ruchu"))
        mani_layout.addWidget(self.sensitivity_slider)

        self.mani_group.setLayout(mani_layout)
        main_layout.addWidget(self.mani_group)

        # Kolumna dla danych RFID
        self.rfid_group = QGroupBox("RFID Data")
        rfid_layout = QVBoxLayout()

        self.rfid_label = QLabel(f"RFID: {self.rfid_data}")
        rfid_layout.addWidget(self.rfid_label)

        self.rfid_group.setLayout(rfid_layout)
        main_layout.addWidget(self.rfid_group)

        self.setLayout(main_layout)

        # Stylizacja UI
        self.setStyleSheet("""
            QLabel {
                font-size: 12px;
                color: #ddd;
            }
            QSlider {
                background: #555;
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

    def init_ros_publishers(self):
        self.publisher = self.node.create_publisher(Float64MultiArray, "/array_topic", 10)  # Zmiana na Float64MultiArray

    def init_ros_subscribers(self):
        self.rfid_subscriber = self.node.create_subscription(String, "/song_of_seas", self.rfid_callback, 10)

    def rfid_callback(self, msg):
        self.rfid_data = msg.data
        self.rfid_label.setText(f"RFID: {self.rfid_data}")

    def set_selected_gamepad(self, gamepad):
        self.selected_gamepad = gamepad
        if self.selected_gamepad:
            self.running = True
            self.gamepad_thread = threading.Thread(target=self.read_gamepad, daemon=True)
            self.gamepad_thread.start()
        else:
            self.running = False

    def read_gamepad(self):
        while self.running and self.selected_gamepad:
            pygame.event.pump()

            # Mapa przycisków do stopni swobody
            button_mapping = {
                0: (0, 1),  # Stopień 1
                1: (0, -1),
                2: (1, 1),  # Stopień 2
                3: (1, -1),
                6: (2, 1),  # Stopień 3
                7: (2, -1),
                9: (3, 1),  # Stopień 4
                10: (3, -1),
            }

            # Mapa HAT (krzyżaka) do stopni swobody
            hat_mapping = {
                (-1, 0): (4, -1),  # Stopień 5
                (1, 0): (4, 1),
                (0, -1): (5, -1),  # Stopień 6
                (0, 1): (5, 1),
            }

            # Odczyt przycisków
            new_values = [0.0] * 6  # Zmiana na wartości zmiennoprzecinkowe
            for button, (index, direction) in button_mapping.items():
                if self.selected_gamepad.get_button(button):
                    new_values[index] = direction * self.sensitivity

            # Odczyt HAT (krzyżaka)
            hat = self.selected_gamepad.get_hat(0)
            if hat in hat_mapping:
                index, direction = hat_mapping[hat]
                new_values[index] = direction * self.sensitivity

            # Ograniczenie zakresu wartości
            for i in range(6):
                new_values[i] = max(-100.0, min(100.0, new_values[i]))  # Zmiana na wartości zmiennoprzecinkowe

            # Aktualizacja wartości i publikacja
            self.current_values = new_values
            self.update_ui()
            self.publish_values()

    def update_ui(self):
        for i, value in enumerate(self.current_values):
            self.movement_labels[i].setText(f"Stopień {i+1}: {value:.2f}")  # Formatowanie do dwóch miejsc po przecinku

    def publish_values(self):
        msg = Float64MultiArray()  # Zmiana na Float64MultiArray
        msg.data = self.current_values
        self.publisher.publish(msg)

    def update_sensitivity(self, value):
        self.sensitivity = float(value)  # Zmiana na wartość zmiennoprzecinkową