from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, 
                            QGroupBox, QGridLayout, QFrame, QSpinBox, QComboBox,
                            QSlider, QScrollArea, QSizePolicy)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QPalette, QFont
from PyQt6.QtCore import QProcess
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray, Int8MultiArray, Int32MultiArray, Float32, Int16
import os
import datetime
import config
import subprocess
import time

class ScienceTab(QWidget):
    def __init__(self, node: Node, serva):
        super().__init__()
        self.node = node
        self.serva = serva
        self.init_ros_subscriptions()
        self.init_ros_publishers()

        # Inicjalizacja danycha
        self.data_directory = "sensor_data"
        os.makedirs(self.data_directory, exist_ok=True)


        # Stany urządzeń
# Stany urządzeń
        #self.basic = [10, 20, 30, 40, 50, 60]

        self.servo_presets = {
            0: [140, 95, 50],   # Presety dla serwa 0 (Serwo 1)
            1: [134, 92, 50],   # Presety dla serwa 1 (Serwo 2)
            2: [63, 104, 145],   # Presety dla serwa 2 (Serwo 3)
            3: [100, 55, 10],   # Presety dla serwa 3 (Serwo 4)
            4: [80, 125, 140],   # Presety dla serwa 4 (Serwo 5)
            5: [57, 98, 140]    # Presety dla serwa 5 (Serwo 6)
        }
        self.basic = [i[0] for i in self.servo_presets.values()]  # Ustawienie wartości podstawowych z presetów
        self.servo_states = self.basic.copy()
        print(f"Basic servo positions: {self.basic}")
        self.pump_states = [False] * 2
        self.led_brightness = 0
        self.led_state = False  # Dodajemy stan LED

        self.drill_state = False
        self.drill_direction = None  # None, 'left', or 'right'
        self.drill_left_button = None
        self.drill_right_button = None

        self.heater_state = False
        self.koszelnik_state = False
        self.pump_buttons = []  # Initialize the pump_buttons list

        self.init_ui()

    def init_ui(self):
        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(10)

        # Lewa kolumna - Czujniki i dane
        left_column = QVBoxLayout()
        left_column.setSpacing(10)
        
        # Kontener z przewijaniem dla lewej kolumny
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll_content = QWidget()
        left_scroll_layout = QVBoxLayout(left_scroll_content)
        left_scroll_layout.setSpacing(10)
        left_scroll_layout.setContentsMargins(5, 5, 5, 5)
        
        # Ramka CO₂
        co2_frame = QGroupBox("Concentration of CO₂")
        co2_layout = QVBoxLayout()
        self.co2_label1 = QLabel("Sensor 1: --- ppm")
        self.co2_label2 = QLabel("Sensor 2: --- ppm")
        self.co2_label1.setFont(QFont('Arial', 12))
        self.co2_label2.setFont(QFont('Arial', 12))
        co2_layout.addWidget(self.co2_label1)
        co2_layout.addWidget(self.co2_label2)
        co2_frame.setLayout(co2_layout)
        left_scroll_layout.addWidget(co2_frame)

        # Ramka Metan
        methane_frame = QGroupBox("Concentration of Metan")
        methane_layout = QVBoxLayout()
        self.methane_label1 = QLabel("Sensor 1: --- ppm")
        self.methane_label2 = QLabel("Sensor 2: --- ppm")
        self.methane_label1.setFont(QFont('Arial', 12))
        self.methane_label2.setFont(QFont('Arial', 12))
        methane_layout.addWidget(self.methane_label1)
        methane_layout.addWidget(self.methane_label2)
        methane_frame.setLayout(methane_layout)
        left_scroll_layout.addWidget(methane_frame)

        # Ramka Temperatura gleby
        temp_frame = QGroupBox("Soil temperature")
        temp_layout = QVBoxLayout()
        self.temp_label = QLabel("--- °C")
        self.temp_label.setFont(QFont('Arial', 12))
        temp_layout.addWidget(self.temp_label)
        temp_frame.setLayout(temp_layout)
        left_scroll_layout.addWidget(temp_frame)

        # Ramka Wilgotność gleby
        humidity_frame = QGroupBox("Soil moisture")
        humidity_layout = QVBoxLayout()
        self.humidity_label = QLabel("--- %")
        self.humidity_label.setFont(QFont('Arial', 12))
        humidity_layout.addWidget(self.humidity_label)
        humidity_frame.setLayout(humidity_layout)
        left_scroll_layout.addWidget(humidity_frame)

        # Ramka Promieniowanie
        radiation_frame = QGroupBox("Radiation")
        radiation_layout = QVBoxLayout()
        self.radiation_label = QLabel("--- mSv/h")
        self.radiation_label.setFont(QFont('Arial', 12))
        radiation_layout.addWidget(self.radiation_label)
        radiation_frame.setLayout(radiation_layout)
        left_scroll_layout.addWidget(radiation_frame)

        # Przycisk do otwierania aplikacji z wykresami
        self.plot_app_button = QPushButton('📈 Otwórz aplikację z wykresami')
        self.plot_app_button.setFont(QFont('Arial', 12))
        self.plot_app_button.setFixedHeight(40)
        self.plot_app_button.clicked.connect(self.launch_plot_app)
        left_scroll_layout.addWidget(self.plot_app_button)


        # Przycisk do otwierania aplikacji z mapą radiacji
        self.radiation_app_button = QPushButton('Otwórz aplikację z mapą radiacji')
        self.radiation_app_button.setFont(QFont('Arial', 12))
        self.radiation_app_button.setFixedHeight(40)
        self.radiation_app_button.clicked.connect(self.launch_radiation_app)
        left_scroll_layout.addWidget(self.radiation_app_button)

        # Przycisk do uruchomienia skryptu i pobrania mapy
        self.generate_map_button = QPushButton('🌍 Generuj mapę radiacji')
        self.generate_map_button.setFont(QFont('Arial', 12))
        self.generate_map_button.setFixedHeight(40)
        self.generate_map_button.clicked.connect(self.generate_radiation_map)
        left_scroll_layout.addWidget(self.generate_map_button)
        
        left_scroll_layout.addStretch()
        left_scroll.setWidget(left_scroll_content)
        left_column.addWidget(left_scroll)

        # Prawa kolumna - Sterowanie
        right_column = QVBoxLayout()
        right_column.setSpacing(10)

        # Kontener z przewijaniem dla prawej kolumny
        right_scroll = QScrollArea()
        right_scroll.setWidgetResizable(True)
        right_scroll_content = QWidget()
        right_scroll_layout = QVBoxLayout(right_scroll_content)
        right_scroll_layout.setSpacing(10)
        right_scroll_layout.setContentsMargins(5, 5, 5, 5)

        # Grupa sterowania serwami
        # Zmieniamy kolejność serw zgodnie z oczekiwaniami
        self.servo_display_order = [
            ("Pojemnik kwadratowy prawy", 2),
            ("Rewolwer prawy", 4),
            ("Pojemnik prawy", 5),
            ("Pojemnik kwadratowy lewy", 1),
            ("Rewolwer lewy", 3),
            ("Pojemnik lewy", 0)
        ]

        # Grupa sterowania serwami
        servo_group = QGroupBox("Sterowanie Serwomechanizmami")
        servo_layout = QGridLayout()

        self.servo_buttons = []  # Przechowuje przyciski presetów
        self.servo_spinboxes = []  # Przechowuje spinboxy

        for row, (label, servo_idx) in enumerate(self.servo_display_order):
            # Etykieta serwa
            servo_layout.addWidget(QLabel(f"{label}:"), row, 0)
            
            # Przyciski presetów
            btn_frame = QFrame()
            btn_layout = QHBoxLayout(btn_frame)
            btn_layout.setContentsMargins(0, 0, 0, 0)
            
            servo_btns = []
            for angle in self.servo_presets[servo_idx]:
                btn = QPushButton(str(angle))
                btn.setFixedWidth(60)
                btn.clicked.connect(self.create_servo_preset_handler(servo_idx, angle, servo_btns))
                btn_layout.addWidget(btn)
                servo_btns.append(btn)
            
            self.servo_buttons.append(servo_btns)
            servo_layout.addWidget(btn_frame, row, 1)
                        
            # SpinBox dla dowolnej wartości
            spinbox = QSpinBox()
            spinbox.setRange(0, 180)
            spinbox.setValue(self.basic[servo_idx])
            spinbox.setFixedWidth(100)
            
            set_btn = QPushButton("Ustaw")
            set_btn.setFixedWidth(150)
            set_btn.clicked.connect(lambda _, idx=servo_idx, sb=spinbox: 
                                self.on_servo_custom_clicked(idx, sb))
            
            servo_layout.addWidget(spinbox, row, 2)
            servo_layout.addWidget(set_btn, row, 3)
            
            self.servo_spinboxes.append(spinbox)

        servo_group.setLayout(servo_layout)
        right_scroll_layout.addWidget(servo_group)
            
        # Grupa sterowania pompami
        pump_group = QGroupBox("Sterowanie Pompami")
        pump_layout = QGridLayout()
        
        for i in range(2):
            pump_layout.addWidget(QLabel(f"Pompa {i+1}:"), i, 0)
            
            on_btn = QPushButton("Włącz")
            off_btn = QPushButton("Wyłącz")
            
            on_btn.clicked.connect(lambda _, idx=i: self.set_pump(idx, True))
            off_btn.clicked.connect(lambda _, idx=i: self.set_pump(idx, False))
            
            pump_layout.addWidget(on_btn, i, 1)
            pump_layout.addWidget(off_btn, i, 2)
            
            self.pump_buttons.append((on_btn, off_btn))
        
        pump_group.setLayout(pump_layout)
        right_scroll_layout.addWidget(pump_group)
        
       # Grupa sterowania LED
        led_group = QGroupBox("Sterowanie LED")
        led_layout = QVBoxLayout()

        # Przycisk Włącz
        self.led_on_btn = QPushButton("Włącz")
        self.led_on_btn.clicked.connect(lambda: self.set_led_state(255))

        # Przycisk Wyłącz
        self.led_off_btn = QPushButton("Wyłącz")
        self.led_off_btn.clicked.connect(lambda: self.set_led_state(0))

        # Układ dla przycisków LED
        led_button_layout = QHBoxLayout()
        led_button_layout.addWidget(QLabel("Sterowanie LED:"))
        led_button_layout.addWidget(self.led_on_btn)
        led_button_layout.addWidget(self.led_off_btn)

        led_layout.addLayout(led_button_layout)
        led_group.setLayout(led_layout)
        right_scroll_layout.addWidget(led_group)

        

        # Grupa zdalnego sterowania
        control_group = QGroupBox("Zdalne sterowanie")
        control_layout = QGridLayout()

        # Wiertło - nagłówek
        control_layout.addWidget(QLabel("Wiertło:"), 2, 0)

        # Kierunek wiertła (lewo/prawo)
        self.drill_left_button = QPushButton("◀")
        self.drill_right_button = QPushButton("▶")
        self.drill_left_button.clicked.connect(lambda: self.set_drill_direction("left"))
        self.drill_right_button.clicked.connect(lambda: self.set_drill_direction("right"))
        self.drill_left_button.setFixedSize(50, 25)  # szerokość, wysokość
        self.drill_right_button.setFixedSize(50, 25)

        control_layout.addWidget(self.drill_left_button, 2, 1)
        control_layout.addWidget(self.drill_right_button, 2, 2)

        # Przyciski ON/OFF wiertła
        self.drill_on_button = QPushButton("🔄 WŁĄCZ")
        self.drill_off_button = QPushButton("WYŁĄCZ")
        self.drill_on_button.clicked.connect(lambda: self.send_control_command(0, True))
        self.drill_off_button.clicked.connect(lambda: self.send_control_command(0, False))
        control_layout.addWidget(self.drill_on_button, 2, 3)
        control_layout.addWidget(self.drill_off_button, 2, 4)

        # Koszelnik (PRZESUNIĘTE do wiersza 3)
        control_layout.addWidget(QLabel("Zdalny Koszelnik:"), 3, 0)
        self.koszelnik_on_button = QPushButton("🍺 WŁĄCZ")
        self.koszelnik_off_button = QPushButton("WYŁĄCZ")
        self.koszelnik_on_button.clicked.connect(lambda: self.send_control_command(1, True))
        self.koszelnik_off_button.clicked.connect(lambda: self.send_control_command(1, False))
        control_layout.addWidget(self.koszelnik_on_button, 3, 3)
        control_layout.addWidget(self.koszelnik_off_button, 3, 4)

        # Grzałka (PRZESUNIĘTE do wiersza 4)
        control_layout.addWidget(QLabel("Grzałka:"), 4, 0)
        self.heater_on_button = QPushButton("🔥 WŁĄCZ")
        self.heater_off_button = QPushButton("WYŁĄCZ")
        self.heater_on_button.clicked.connect(lambda: self.send_control_command(2, True))
        self.heater_off_button.clicked.connect(lambda: self.send_control_command(2, False))
        control_layout.addWidget(self.heater_on_button, 4, 3)
        control_layout.addWidget(self.heater_off_button, 4, 4)


        control_group.setLayout(control_layout)
        right_scroll_layout.addWidget(control_group)

        # W apply_styles() dodaj na końcu:
        self.update_button_style(self.drill_on_button, config.BUTTON_DEFAULT_COLOR)
        self.update_button_style(self.drill_off_button, config.BUTTON_OFF_COLOR)
        self.update_button_style(self.koszelnik_on_button, config.BUTTON_DEFAULT_COLOR)
        self.update_button_style(self.koszelnik_off_button, config.BUTTON_OFF_COLOR)
        self.update_button_style(self.heater_on_button, config.BUTTON_DEFAULT_COLOR)
        self.update_button_style(self.heater_off_button, config.BUTTON_OFF_COLOR)
        self.update_button_style(self.led_on_btn, config.BUTTON_DEFAULT_COLOR)
        self.update_button_style(self.led_off_btn, config.BUTTON_OFF_COLOR)
        
        right_scroll_layout.addStretch()
        right_scroll.setWidget(right_scroll_content)
        right_column.addWidget(right_scroll)
        
        # Dodanie kolumn do głównego layoutu
        main_layout.addLayout(left_column, stretch=1)
        main_layout.addLayout(right_column, stretch=2)
        
        self.setLayout(main_layout)
        self.apply_styles()
        

    def init_ros_subscriptions(self):
        self.node.create_subscription(Float32MultiArray, 'CO2_publisher', self.co2_callback, 10)
        self.node.create_subscription(Float32, 'Temp_Publisher', self.temp_callback, 10)
        self.node.create_subscription(Float32, 'Humidity_Publisher', self.humidity_callback, 10)
        self.node.create_subscription(Float32MultiArray, 'Methane_publisher', self.methane_callback, 10)
        self.node.create_subscription(Float32, 'Radiation_Publisher', self.radiation_callback, 10)

    def init_ros_publishers(self):
        self.servo_publisher = self.node.create_publisher(Int32MultiArray, '/servos_urc_control', 10)
        self.pump_publisher = self.node.create_publisher(Int8MultiArray, '/pumps_urc_control', 10)
        self.led_publisher = self.node.create_publisher(Int16, '/led_urc_control', 10)
        self.koszelnik_publisher = self.node.create_publisher(Int8MultiArray, '/ESP32_GIZ/output_state_topic', 10)

    def co2_callback(self, msg):
        if len(msg.data) >= 2:
            self.co2_label1.setText(f"Senor 1: {msg.data[0]:.1f} ppm")
            self.co2_label2.setText(f"Sensor 2: {msg.data[1]:.1f} ppm")
            
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            with open(f"{self.data_directory}/co2.txt", "a") as f:
                f.write(f"{timestamp}, {msg.data[0]:.1f}, {msg.data[1]:.1f}\n")

    def methane_callback(self, msg):
        if len(msg.data) >= 2:
            self.methane_label1.setText(f"Sensor 1: {msg.data[0]:.2f} ppm")
            self.methane_label2.setText(f"Sensor 2: {msg.data[1]:.2f} ppm")
            
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            with open(f"{self.data_directory}/methane.txt", "a") as f:
                f.write(f"{timestamp}, {msg.data[0]:.2f}, {msg.data[1]:.2f}\n")

    def temp_callback(self, msg):
        temp_value = msg.data
        self.temp_label.setText(f"{temp_value:.1f} °C")
        
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(f"{self.data_directory}/soil_temp.txt", "a") as f:
            f.write(f"{timestamp}, {temp_value:.1f}\n")

    def humidity_callback(self, msg):
        humidity_value = msg.data
        self.humidity_label.setText(f"{humidity_value:.1f} %")
        
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(f"{self.data_directory}/soil_humidity.txt", "a") as f:
            f.write(f"{timestamp}, {humidity_value:.1f}\n")

    def radiation_callback(self, msg):
        radiation_value = msg.data
        self.radiation_label.setText(f"{radiation_value:.1f} mSv/h")
        
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(f"{self.data_directory}/radiation.txt", "a") as f:
            f.write(f"{timestamp}, {radiation_value:.1f}\n")


    def on_servo_custom_clicked(self, servo_idx, spinbox):
        # Zresetuj style wszystkich przycisków presetów dla tego serwa
        for btn in self.servo_buttons[servo_idx]:
            btn.setStyleSheet("")
        
        # Wyślij komendę
        self.set_servo(servo_idx)

    
    def create_servo_preset_handler(self, servo_idx, angle, buttons):
        """Tworzy handler dla przycisku presetów z odpowiednim kolorowaniem"""
        def handler():
            # Zresetuj style wszystkich przycisków dla tego serwa
            for btn in buttons:
                btn.setStyleSheet("")
            
            # Podświetl kliknięty przycisk
            self.sender().setStyleSheet(f"background-color: {config.BUTTON_ON_COLOR};")
            
            # Ustaw wartość i wyślij komendę
            self.servo_spinboxes[servo_idx].setValue(angle)
            self.set_servo(servo_idx)
        return handler

    def on_servo_custom_clicked(self, servo_idx, spinbox):
        """Obsługa niestandardowej wartości z SpinBoxa"""
        # Zresetuj style wszystkich przycisków presetów dla tego serwa
        for btn in self.servo_buttons[servo_idx]:
            btn.setStyleSheet("")
        
        # Ustaw i wyślij wartość
        self.servo_states[servo_idx] = spinbox.value()
        self.set_servo(servo_idx)

    def set_servo_custom(self, index):
        self.set_servo(index)

    def set_servo(self, index=None):
        if index is not None:
            angle = self.servo_spinboxes[index].value()
            self.servo_states[index] = angle

        if self.node.communication_mode == 'ROS2':
            msg = Int32MultiArray()
            msg.data = self.servo_states
            self.servo_publisher.publish(msg)

        elif self.node.communication_mode == 'SATEL':
            # Wysyłamy tylko 8-bitową wersję (wartości 0-255!)
            self.node.send_serial_frame("SS", *[val & 0xFF for val in self.servo_states])
        

    def set_pump(self, index, state):
        self.pump_states[index] = state

        if self.node.communication_mode == 'ROS2':
            msg = Int8MultiArray()
            msg.data = [int(pump) for pump in self.pump_states]
            self.pump_publisher.publish(msg)

        elif self.node.communication_mode == 'SATEL':
            # Kodowanie 2 pomp w 1 bajcie
            pumps_byte = 0
            for i, val in enumerate(self.pump_states):
                pumps_byte |= (val & 0x01) << i
            self.node.send_serial_frame("SP", pumps_byte)
        
        on_btn, off_btn = self.pump_buttons[index]
        on_btn.setStyleSheet("background-color: green;" if state else "")
        off_btn.setStyleSheet("background-color: red;" if not state else "")

    def led_slider_changed(self, value):
        self.led_spinbox.setValue(value)

    def led_spinbox_changed(self, value):
        self.led_slider.setValue(value)

    def set_led_state(self, brightness):
        self.led_brightness = brightness
        self.led_state = brightness > 0

        if self.node.communication_mode == 'ROS2':
            msg = Int16()
            msg.data = brightness
            self.led_publisher.publish(msg)
        elif self.node.communication_mode == 'SATEL':
            self.node.send_serial_frame("SL", brightness & 0xFF)
            
        self.update_button_style(
            self.led_on_btn, config.BUTTON_ON_COLOR if self.led_state else config.BUTTON_DEFAULT_COLOR)
        self.update_button_style(
            self.led_off_btn, config.BUTTON_OFF_COLOR if not self.led_state else config.BUTTON_DEFAULT_COLOR)


    
    def send_control_command(self, device_id, state):
        """Wysyła komendę sterującą dla wybranego urządzenia
        0 - wiertło, 1 - koszelnik, 2 - grzałka"""
        msg = Int8MultiArray()
        # Zachowaj obecne stany wszystkich urządzeń
        msg.data = [int(self.drill_state), int(self.koszelnik_state), int(self.heater_state)]
        # Zaktualizuj tylko wybrane urządzenie
        msg.data[device_id] = int(state)
        
        # Aktualizacja stanów
        if device_id == 0:
            self.drill_state = state
        elif device_id == 1:
            self.koszelnik_state = state
        elif device_id == 2:
            self.heater_state = state
        
        # Aktualizacja przycisków
        self.update_button_states()
        if self.node.communication_mode == 'ROS2':
            self.koszelnik_publisher.publish(msg)
        elif self.node.communication_mode == 'SATEL':
            byte = (msg.data[0] << 0) | (msg.data[1] << 1) | (msg.data[2] << 2)
            self.node.send_serial_frame("GK", byte)

    def update_button_states(self):
        """Aktualizuje style wszystkich przycisków zgodnie z aktualnymi stanami"""
        # Wiertło
        self.update_button_style(self.drill_on_button, config.BUTTON_ON_COLOR if self.drill_state else config.BUTTON_DEFAULT_COLOR)
        self.update_button_style(self.drill_off_button, config.BUTTON_OFF_COLOR if not self.drill_state else config.BUTTON_DEFAULT_COLOR)
        
        # Koszelnik
        self.update_button_style(self.koszelnik_on_button, config.BUTTON_ON_COLOR if self.koszelnik_state else config.BUTTON_DEFAULT_COLOR)
        self.update_button_style(self.koszelnik_off_button, config.BUTTON_OFF_COLOR if not self.koszelnik_state else config.BUTTON_DEFAULT_COLOR)
        
        # Grzałka
        self.update_button_style(self.heater_on_button, config.BUTTON_ON_COLOR if self.heater_state else config.BUTTON_DEFAULT_COLOR)
        self.update_button_style(self.heater_off_button, config.BUTTON_OFF_COLOR if not self.heater_state else config.BUTTON_DEFAULT_COLOR)
        
        # LED
        self.update_button_style(self.led_on_btn, config.BUTTON_ON_COLOR if self.led_state else config.BUTTON_DEFAULT_COLOR)
        self.update_button_style(self.led_off_btn, config.BUTTON_OFF_COLOR if not self.led_state else config.BUTTON_DEFAULT_COLOR)

    def set_drill_direction(self, direction):
        if self.drill_state:
            self.send_control_command(0, 0)
            time.sleep(0.5)
        if direction == "left":
            self.drill_direction = "left"
            self.serva.set_servo_position(2, 100)
            self.drill_left_button.setStyleSheet("background-color: green;")
            self.drill_right_button.setStyleSheet("")
            self.drill_right_button.setChecked(False)
        elif direction == "right":
            self.drill_direction = "right"
            self.serva.set_servo_position(2, 10)
            self.drill_right_button.setStyleSheet("background-color: green;")
            self.drill_left_button.setStyleSheet("")
            self.drill_left_button.setChecked(False)


    def launch_plot_app(self):
        try:
            subprocess.Popen(["python", "wykresy.py"])
        except Exception as e:
            self.node.get_logger().error(f"Nie udało się uruchomić aplikacji z wykresami: {str(e)}")
    
    def launch_radiation_app(self):
        try:
            # Launch the Tkinter-based radiation map window
            radiation_map_path = os.path.join(os.path.dirname(__file__), 'GPS/radiation_map.py')
            QProcess.startDetached('python', [radiation_map_path])
        except Exception as e:
            self.node.get_logger().error(f"Nie udało się uruchomić aplikacji z radiacją: {str(e)}")

    def generate_radiation_map(self):
        try:
            generate_html_map = os.path.join(os.path.dirname(__file__), 'GPS/generate_radiation_map.py')
            QProcess.startDetached('python', [generate_html_map])
        except Exception as e:
            self.node.get_logger().error(f"Nie udało się uruchomić aplikacji: {str(e)}")

    def update_button_style(self, button, color):
        button.setStyleSheet(f"background-color: {color};")
        button.update()

    def apply_styles(self):
        self.setStyleSheet("""
            QWidget {
                background-color: #333;
                color: #ddd;
                font-size: 12px;
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
                padding: 0 3px;
            }
            QPushButton {
                background-color: #444;
                border: 2px solid #555;
                border-radius: 6px;
                padding: 5px;
            }
            QPushButton:hover {
                background-color: #555;
            }
            QSpinBox, QSlider {
                background-color: #444;
                color: #ddd;
                border: 1px solid #555;
                padding: 3px;
            }
            QLabel {
                padding: 3px;
            }
            QScrollArea {
                border: none;
            }
        """)
        
        # Specjalny styl dla przycisku wykresów
        self.plot_app_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                font-weight: bold;
                border: 2px solid #45a049;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)