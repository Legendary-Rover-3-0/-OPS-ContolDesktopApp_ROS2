import threading
import time
import pygame
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, 
                            QGroupBox, QGridLayout, QTextEdit, QScrollArea, 
                            QFrame, QPushButton)
from PyQt6.QtCore import Qt
from rclpy.node import Node
from std_msgs.msg import String, Float32, Float64MultiArray
from geometry_msgs.msg import Twist
import config


class ManipulatorTab(QWidget):
    def __init__(self, node: Node, gamepads):
        super().__init__()
        self.node = node
        self.gamepads = gamepads
        self.selected_gamepad = None
        self.running = False
        self.is_gamepad_active = False
        self.gamepad_thread = None

        # Inicjalizacja wartości dla 6 stopni swobody
        self.current_values = [0.0] * 6
        self.sensitivity = config.MANI_DEFAULT_VALUE

        # Inicjalizacja danych RFID
        self.rfid_data = "Brak danych"
        
        # Inicjalizacja danych temperatury
        self.cpu_temp = 0.0
        self.gpu_temp = 0.0
        
        # Definicja presetów czułości
        self.sensitivity_presets = [
            ["Full Sensitivity", 100, 100, 100, 100, 100, 100],
            ["Klawiatura", 90, 100, 60, 80, 75, 90],
        ]

        self.init_ui()
        self.init_ros_publishers()
        self.init_ros_subscribers()

    def init_ui(self):
        main_layout = QVBoxLayout()

        # Sekcja temperatury
        temp_group = QGroupBox("Temperatury")
        temp_layout = QHBoxLayout()
        
        self.cpu_temp_label = QLabel("CPU: --.-- °C")
        self.gpu_temp_label = QLabel("GPU: --.-- °C")
        
        temp_layout.addWidget(self.cpu_temp_label)
        temp_layout.addWidget(self.gpu_temp_label)
        temp_group.setLayout(temp_layout)
        main_layout.addWidget(temp_group)

        # Globalny suwak czułości
        global_sens_group = QGroupBox("Globalna czułość")
        global_sens_layout = QHBoxLayout()
        
        self.global_sensitivity_slider = QSlider(Qt.Orientation.Horizontal)
        self.global_sensitivity_slider.setMinimum(1)
        self.global_sensitivity_slider.setMaximum(100)
        self.global_sensitivity_slider.setValue(int(self.sensitivity))
        self.global_sensitivity_slider.valueChanged.connect(self.update_global_sensitivity)
        
        self.global_sens_value_label = QLabel(f"{self.sensitivity}")
        self.global_sens_value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        global_sens_layout.addWidget(QLabel("Czułość wszystkich stopni:"))
        global_sens_layout.addWidget(self.global_sensitivity_slider)
        global_sens_layout.addWidget(self.global_sens_value_label)
        global_sens_group.setLayout(global_sens_layout)
        main_layout.addWidget(global_sens_group)

        # Sekcja presetów czułości
        presets_group = QGroupBox("Presety czułości")
        presets_layout = QHBoxLayout()
        
        for preset in self.sensitivity_presets:
            btn = QPushButton(preset[0])
            btn.setToolTip(" | ".join(f"{val}" for val in preset[1:]))
            btn.clicked.connect(lambda _, p=preset: self.apply_preset(p))
            presets_layout.addWidget(btn)
        
        presets_group.setLayout(presets_layout)
        main_layout.addWidget(presets_group)

        # Kolumna dla manipulatora
        self.mani_group = QGroupBox("Manipulator Control")
        mani_layout = QGridLayout()

        # Inicjalizacja suwaków dla każdego stopnia swobody
        self.sensitivities = [config.MANI_DEFAULT_VALUE] * 6
        self.sensitivity_sliders = []

        # Etykiety i suwaki dla każdego stopnia swobody
        self.movement_labels = []
        degree_names = [
            "Podstawa", 
            "Ramie (dół)", 
            "Ramię (góra)", 
            "Nadgarstek", 
            "Chwytak obrót",
            "Chwytak zacisk"
        ]
        for i in range(6):
            name = degree_names[i] if i < len(degree_names) else f"Stopień {i+1}"
            label = QLabel(f"{name}: {i+1}: 0.0")
            self.movement_labels.append(label)
            mani_layout.addWidget(label, i, 0)

            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setMinimum(1)
            slider.setMaximum(100)
            slider.setValue(int(self.sensitivities[i]))
            slider.valueChanged.connect(lambda value, idx=i: self.update_sensitivity(idx, value))

            slider.setFixedWidth(200)
            self.sensitivity_sliders.append(slider)
            mani_layout.addWidget(QLabel(f"Czułość Stopnia {i+1}"), i, 1)
            mani_layout.addWidget(slider, i, 2)

            value_label = QLabel(f"{self.sensitivities[i]}")
            value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.sensitivity_sliders[i].valueChanged.connect(lambda value, lbl=value_label: lbl.setText(f"{value}"))
            mani_layout.addWidget(value_label, i, 3)

        self.mani_group.setLayout(mani_layout)
        main_layout.addWidget(self.mani_group)

        # Sekcja RFID
        self.rfid_group = QGroupBox("RFID Data")
        rfid_layout = QVBoxLayout()
        
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.Shape.NoFrame)
        
        self.rfid_text = QTextEdit()
        self.rfid_text.setReadOnly(True)
        self.rfid_text.setPlainText(self.rfid_data)
        self.rfid_text.setStyleSheet("""
            QTextEdit {
                background-color: #3a3a3a;
                color: #ddd;
                border: 1px solid #555;
                border-radius: 4px;
                padding: 5px;
                font-size: 14px;
            }
        """)
        self.rfid_text.setMaximumHeight(300)
        
        scroll_area.setWidget(self.rfid_text)
        rfid_layout.addWidget(scroll_area)
        
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
                min-width: 200px;
                margin: 0 10px;
            }
            QSlider::groove:horizontal {
                background: #666;
                height: 8px;
                border-radius: 4px;
                margin: 0px;
            }
            QSlider::handle:horizontal {
                background: #ddd;
                width: 18px;
                margin: -5px 0;
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
            QPushButton {
                background-color: #555;
                color: #ddd;
                border: 1px solid #666;
                border-radius: 4px;
                padding: 5px;
                min-width: 80px;
            }
            QPushButton:hover {
                background-color: #666;
            }
            QPushButton:pressed {
                background-color: #777;
            }
        """)

    def apply_preset(self, preset):
        """Stosuje wybrany preset czułości"""
        preset_name = preset[0]
        sensitivity_values = preset[1:]
        
        for i, value in enumerate(sensitivity_values):
            if i < len(self.sensitivity_sliders):
                self.sensitivity_sliders[i].setValue(value)
                self.sensitivities[i] = float(value)

    def add_new_preset(self):
        """Dodaje nowy preset na podstawie aktualnych ustawień"""
        # Tutaj można dodać dialog do wprowadzenia nazwy nowego presetu
        # Na razie używamy domyślnej nazwy
        preset_name = f"Custom {len(self.sensitivity_presets) + 1}"
        new_preset = [preset_name] + [int(slider.value()) for slider in self.sensitivity_sliders]
        
        self.sensitivity_presets.append(new_preset)
        
        # Dodaj nowy przycisk
        btn = QPushButton(preset_name)
        btn.setToolTip(" | ".join(f"{val}" for val in new_preset[1:]))
        btn.clicked.connect(lambda _, p=new_preset: self.apply_preset(p))
        
        # Znajdź layout z przyciskami presetów
        presets_group = self.findChild(QGroupBox, "Presety czułości")
        if presets_group:
            layout = presets_group.layout()
            # Wstaw nowy przycisk przed przyciskiem "+"
            layout.insertWidget(layout.count() - 1, btn)
            
            self.node.get_logger().info(f"Added new preset: {new_preset}")


    def init_ros_publishers(self):
        self.publisher = self.node.create_publisher(Float64MultiArray, "/array_topic", 10)

    def init_ros_subscribers(self):
        self.node.create_subscription(String, "/RFID/string_data", self.rfid_callback, 10)
        # Subskrypcje temperatury
        self.node.create_subscription(Float32, "/jetson/temperature/cpu", self.cpu_temp_callback, 10)
        self.node.create_subscription(Float32, "/jetson/temperature/gpu", self.gpu_temp_callback, 10)

    def cpu_temp_callback(self, msg: Float32):
        self.cpu_temp = msg.data
        self.cpu_temp_label.setText(f"CPU: {self.cpu_temp:.2f} °C")

    def gpu_temp_callback(self, msg: Float32):
        self.gpu_temp = msg.data
        self.gpu_temp_label.setText(f"GPU: {self.gpu_temp:.2f} °C")

    def rfid_callback(self, msg: String):
        self.rfid_data = msg.data
        self.rfid_text.setPlainText(f"RFID: {self.rfid_data}")
        self.rfid_text.verticalScrollBar().setValue(
            self.rfid_text.verticalScrollBar().maximum()
        )

    def update_global_sensitivity(self, value):
        # Ustawia tę samą czułość dla wszystkich stopni
        for i in range(6):
            self.sensitivity_sliders[i].setValue(value)
            self.sensitivities[i] = float(value)
        self.global_sens_value_label.setText(f"{value}")

    def set_selected_gamepad(self, gamepad):
        self.selected_gamepad = gamepad
        if self.selected_gamepad and (self.gamepad_thread is None or not self.gamepad_thread.is_alive()):
            self.running = True
            self.gamepad_thread = threading.Thread(target=self.read_gamepad)
            self.gamepad_thread.start()

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
                9: (3, 1),  # Stopień 4 nadgarstek
                10: (3, -1),
            }

            # Mapa HAT (krzyżaka) do stopni swobody
            # hat_mapping = {
            #     (1, 0): (3, 1),  # Stopień 3 gora dol 3
            #     (-1, 0): (3, -1),
            #     (0, 1): (5, -1),  # Stopień 2 przod tyl 5
            #     (0, -1): (5, 1),
            # }

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
            if hat[1] == -1:  
                new_values[1] += self.sensitivities[1]
            elif hat[1] == 1:  
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
        # Lista własnych nazw dla każdego stopnia
        degree_names = [
            "Nadgarstek", 
            "Chwytak zacisk",
            "Podstawa", 
            "Ramię (góra)", 
            "Chwytak obrót",
            "Ramie (dół)"
        ]
        
        for i, value in enumerate(self.current_values):
            # Upewnij się, że lista nazw jest wystarczająco długa
            name = degree_names[i] if i < len(degree_names) else f"Stopień {i+1}"
            self.movement_labels[i].setText(f"{name}: {value:.2f}")# Formatowanie do dwóch miejsc po przecinku

    def publish_values(self):
        msg = Float64MultiArray()
        
        # Załóżmy, że np. wartości przypisujemy tak:
        # Stopień 1 -> linear.x
        # Stopień 2 -> linear.y
        # Stopień 3 -> linear.z
        # Stopień 4 -> angular.x
        # Stopień 5 -> angular.y
        # Stopień 6 -> angular.z

        # msg.linear.x = self.current_values[0] / 100.0
        # msg.linear.y = self.current_values[1] / 100.0
        # msg.linear.z = self.current_values[2] / 100.0
        # msg.angular.x = self.current_values[3] / 100.0
        # msg.angular.y = self.current_values[4] / 100.0
        # msg.angular.z = self.current_values[5] / 100.0

        msg.data = [0.0] * 6 
        msg.data[0] = self.current_values[0]
        msg.data[1] = self.current_values[1]
        msg.data[2] = self.current_values[2]
        msg.data[3] = self.current_values[3]
        msg.data[4] = self.current_values[4]
        msg.data[5] = self.current_values[5]


        # Wyślij ramkę
        if self.node.communication_mode == 'ROS2':
            self.publisher.publish(msg)

        elif self.node.communication_mode == 'SATEL':
            self.node.send_serial_frame("MN",
                                self.node.float_to_byte_100(msg.data[0]),
                                self.node.float_to_byte_100(msg.data[1]),
                                self.node.float_to_byte_100(msg.data[2]),
                                self.node.float_to_byte_100(msg.data[3]),
                                self.node.float_to_byte_100(msg.data[4]),
                                self.node.float_to_byte_100(msg.data[5])
                                )


    def publish_empty_values(self):
        if self.node.communication_mode == 'ROS2':
            msg = Float64MultiArray()
            msg.data = [0.0] * 6 
            self.publisher.publish(msg)
        elif self.node.communication_mode == 'SATEL':
            self.node.send_serial_frame("MN",127, 127, 127, 127, 127, 127)

    def update_sensitivity(self, index, value):
        self.sensitivities[index] = float(value)  # Aktualizacja konkretnego stopnia