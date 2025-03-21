import threading
import time
import pygame
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QGroupBox, QGridLayout
from PyQt6.QtCore import Qt
from rclpy.node import Node
from std_msgs.msg import String  # Dodano String dla danych RFID
from geometry_msgs.msg import Twist
import config


class ManipulatorTab(QWidget):
    def __init__(self, node: Node, gamepads):
        super().__init__()
        self.node = node
        self.gamepads = gamepads
        self.selected_gamepad = None
        self.running = False
        self.is_gamepad_active = False  # Dodana flaga kontrolująca aktywność gamepada
        self.gamepad_thread = None

        # Inicjalizacja wartości dla 6 stopni swobody
        self.current_values = [0.0] * 6  # Zmiana na wartości zmiennoprzecinkowe
        self.sensitivity = config.MANI_DEFAULT_VALUE  # Domyślny krok ruchu, regulowany suwakiem

        # Inicjalizacja danych RFID
        self.rfid_data = "Brak danych"

        self.init_ui()
        self.init_ros_publishers()
        self.init_ros_subscribers()

    def init_ui(self):
        main_layout = QVBoxLayout()  # Główny układ pionowy

        # Kolumna dla manipulatora
        self.mani_group = QGroupBox("Manipulator Control")
        mani_layout = QGridLayout()

        # Inicjalizacja suwaków dla każdego stopnia swobody
        self.sensitivities = [config.MANI_DEFAULT_VALUE] * 6  # Domyślnie 50 dla każdego stopnia
        self.sensitivity_sliders = []

        # Etykiety i suwaki dla każdego stopnia swobody
        self.movement_labels = []
        for i in range(6):
            label = QLabel(f"Stopień {i+1}: 0.0")
            self.movement_labels.append(label)
            mani_layout.addWidget(label, i, 0)

            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setMinimum(1)
            slider.setMaximum(100)
            slider.setValue(int(self.sensitivities[i]))
            slider.valueChanged.connect(lambda value, idx=i: self.update_sensitivity(idx, value))  # Przekazujemy indeks

            # Ustawienie stałej szerokości suwaka
            slider.setFixedWidth(200)  # Możesz dostosować szerokość do swoich potrzeb

            self.sensitivity_sliders.append(slider)
            mani_layout.addWidget(QLabel(f"Czułość Stopnia {i+1}"), i, 1)
            mani_layout.addWidget(slider, i, 2)

            # Dodanie wartości liczbowej obok suwaka
            value_label = QLabel(f"{self.sensitivities[i]}")
            value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.sensitivity_sliders[i].valueChanged.connect(lambda value, lbl=value_label: lbl.setText(f"{value}"))
            mani_layout.addWidget(value_label, i, 3)

        self.mani_group.setLayout(mani_layout)
        main_layout.addWidget(self.mani_group)

        # Sekcja RFID na dole
        self.rfid_group = QGroupBox("RFID Data")
        rfid_layout = QHBoxLayout()

        self.rfid_label = QLabel(f"RFID: {self.rfid_data}")
        self.rfid_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        rfid_layout.addWidget(self.rfid_label)

        self.rfid_group.setLayout(rfid_layout)
        main_layout.addWidget(self.rfid_group)

        self.setLayout(main_layout)

        # Stylizacja UI
        self.setStyleSheet("""
            QLabel {
                font-size: 14px;
                color: #ddd;
            }
            QSlider {
                background: #555;
                min-width: 200px;  /* Stała szerokość suwaka */
                margin: 0 10px;    /* Marginesy dla suwaka */
            }
            QSlider::groove:horizontal {
                background: #666;
                height: 8px;
                border-radius: 4px;
                margin: 0px;      /* Usunięcie marginesów w groove */
            }
            QSlider::handle:horizontal {
                background: #ddd;
                width: 18px;
                margin: -5px 0;    /* Dostosowanie marginesów dla uchwytu */
                border-radius: 9px;
            }
            QGroupBox {
                font-size: 16px;
                font-weight: bold;
                margin-top: 10px;
                background-color: #444;
                color: #ddd;
                border: 2px solid #555;
                border-radius: 5px;
                padding: 10px;
            }
            QWidget {
                background-color: #333;
            }
        """)

    def init_ros_publishers(self):
        self.publisher = self.node.create_publisher(Twist, "/array_topic", 10)

    def init_ros_subscribers(self):
        self.node.create_subscription(String, "/RFID/string_data", self.rfid_callback, 10)

    def rfid_callback(self, msg: String):
        self.rfid_data = msg.data
        self.rfid_label.setText(f"RFID: {self.rfid_data}")

    def set_selected_gamepad(self, gamepad):
        self.selected_gamepad = gamepad
        if self.selected_gamepad and (self.gamepad_thread is None or not self.gamepad_thread.is_alive()):
            self.running = True
            self.gamepad_thread = threading.Thread(target=self.read_gamepad)#, daemon=True)
            self.gamepad_thread.start()
        # else:
        #     self.running = False

    def read_gamepad(self):
        while self.running and self.selected_gamepad:
            if not self.is_gamepad_active:  # Sprawdzamy, czy gamepad jest aktywny
                for _ in range(5):
                    self.publish_empty_values()
                    time.sleep(0.05)
                self.running = False
                break

            pygame.event.pump()

            # Mapa przycisków do stopni swobody
            button_mapping = {
                0: (0, 1),  # Stopień 1 dół
                1: (0, -1),
                2: (4, 1),  # Stopień 5 chwytak obrot
                3: (4, -1),
                6: (5, -1),  # Stopień 6 chwytak zacisk
                7: (5, 1),
                9: (3, -1),  # Stopień 4 chwytak gora dol
                10: (3, 1),
            }

            # Mapa HAT (krzyżaka) do stopni swobody
            hat_mapping = {
                (-1, 0): (2, 1),  # Stopień 3 gora dol
                (1, 0): (2, -1),
                (0, -1): (1, -1),  # Stopień 2 przod tyl
                (0, 1): (1, 1),
            }

            # Odczyt przycisków
            new_values = [0.0] * 6
            for button, (index, direction) in button_mapping.items():
                if self.selected_gamepad.get_button(button):
                    new_values[index] += direction * self.sensitivities[index]  # INDYWIDUALNA czułość

            # Odczyt HAT (krzyżaka)
            hat = self.selected_gamepad.get_hat(0)

            # Sprawdzanie osi X
            if hat[0] == -1:  
                new_values[2] += self.sensitivities[2]
            elif hat[0] == 1:  
                new_values[2] -= self.sensitivities[2]

            # Sprawdzanie osi Y
            if hat[1] == 1:  
                new_values[1] += self.sensitivities[1]
            elif hat[1] == -1:  
                new_values[1] -= self.sensitivities[1]

            # Ograniczenie zakresu wartości
            for i in range(6):
                new_values[i] = max(-100.0, min(100.0, new_values[i]))

            # Aktualizacja wartości i publikacja
            if self.current_values != new_values:
                self.current_values = new_values
                self.update_ui()
                self.publish_values()

    def update_ui(self):
        for i, value in enumerate(self.current_values):
            self.movement_labels[i].setText(f"Stopień {i+1}: {value:.2f}")  # Formatowanie do dwóch miejsc po przecinku

    def publish_values(self):
        msg = Twist()
        
        # Załóżmy, że np. wartości przypisujemy tak:
        # Stopień 1 -> linear.x
        # Stopień 2 -> linear.y
        # Stopień 3 -> linear.z
        # Stopień 4 -> angular.x
        # Stopień 5 -> angular.y
        # Stopień 6 -> angular.z

        msg.linear.x = self.current_values[0] / 100.0
        msg.linear.y = self.current_values[1] / 100.0
        msg.linear.z = self.current_values[2] / 100.0
        msg.angular.x = self.current_values[3] / 100.0
        msg.angular.y = self.current_values[4] / 100.0
        msg.angular.z = self.current_values[5] / 100.0

        self.publisher.publish(msg)

    def publish_empty_values(self):
        msg = Twist()
        self.publisher.publish(msg)

    def update_sensitivity(self, index, value):
        self.sensitivities[index] = float(value)  # Aktualizacja konkretnego stopnia