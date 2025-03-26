from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox, QGridLayout, QFrame
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QPalette, QFont
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray, Int8MultiArray
import os
import datetime
import config
from PyQt6.QtWidgets import QApplication
import subprocess


class ScienceTab(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        
        self.init_ros_subscriptions()
        self.init_ros_publishers()

        # Initialize data history
        self.max_data_points = 100
        self.temperature_history = [[] for _ in range(4)]
        self.mass_history = [[] for _ in range(4)]
        self.soil_moisture_history = [[] for _ in range(4)]
        self.time_steps = 0
        self.update_interval = 5
        self.update_counter = 0

        # Create data directory
        self.data_directory = "sensor_data"
        if not os.path.exists(self.data_directory):
            os.makedirs(self.data_directory)

        # Servo states
        self.servo_states = [0] * 4
        self.drill_state = 0
        self.heater_state = 0

        self.init_ui()

    def init_ui(self):
        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(15)

        # Left column - Sensor Data (now takes most space)
        left_column = QVBoxLayout()
        left_column.setSpacing(12)

        # Temperature Group
        self.temperature_group = QGroupBox("🌡️ Temperature Sensors")
        self.temperature_group.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        temp_layout = QGridLayout()
        temp_layout.setSpacing(10)
        self.temperature_labels = []
        for i in range(4):
            frame = QFrame()
            frame.setFrameShape(QFrame.Shape.StyledPanel)
            frame.setLineWidth(1)
            frame_layout = QVBoxLayout(frame)
            
            label = QLabel(f"Sensor {i+1}:\n--- °C")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setFont(QFont('Arial', 11))
            self.temperature_labels.append(label)
            frame_layout.addWidget(label)
            
            temp_layout.addWidget(frame, i//2, i%2)
        self.temperature_group.setLayout(temp_layout)
        left_column.addWidget(self.temperature_group)

        # Mass Group
        self.mass_group = QGroupBox("⚖️ Mass Sensors")
        self.mass_group.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        mass_layout = QGridLayout()
        mass_layout.setSpacing(10)
        self.mass_labels = []
        for i in range(4):
            frame = QFrame()
            frame.setFrameShape(QFrame.Shape.StyledPanel)
            frame.setLineWidth(1)
            frame_layout = QVBoxLayout(frame)
            
            label = QLabel(f"Sensor {i+1}:\n--- g")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setFont(QFont('Arial', 11))
            self.mass_labels.append(label)
            frame_layout.addWidget(label)
            
            mass_layout.addWidget(frame, i//2, i%2)
        self.mass_group.setLayout(mass_layout)
        left_column.addWidget(self.mass_group)

        # Soil Moisture Group
        self.soil_moisture_group = QGroupBox("💧 Soil Moisture Sensors")
        self.soil_moisture_group.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        soil_layout = QGridLayout()
        soil_layout.setSpacing(10)
        self.soil_moisture_labels = []
        for i in range(4):
            frame = QFrame()
            frame.setFrameShape(QFrame.Shape.StyledPanel)
            frame.setLineWidth(1)
            frame_layout = QVBoxLayout(frame)
            
            label = QLabel(f"Sensor {i+1}:\n---")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setFont(QFont('Arial', 11))
            self.soil_moisture_labels.append(label)
            frame_layout.addWidget(label)
            
            soil_layout.addWidget(frame, i//2, i%2)
        self.soil_moisture_group.setLayout(soil_layout)
        left_column.addWidget(self.soil_moisture_group)

        # Plot button at the bottom
        self.plot_app_button = QPushButton('📈 Open Plot App')
        self.plot_app_button.setFont(QFont('Arial', 12))
        self.plot_app_button.setFixedHeight(40)
        self.plot_app_button.clicked.connect(self.launch_plot_app)
        left_column.addWidget(self.plot_app_button, alignment=Qt.AlignmentFlag.AlignBottom)
        left_column.addStretch()

        # Right column - Controls (fixed width)
        right_column = QWidget()
        right_column.setLayout(QVBoxLayout())
        right_column.layout().setSpacing(12)
        right_column.setMinimumWidth(450)

        # Servo Control Group
        servo_group = QGroupBox("Servo Control")
        servo_group.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        servo_layout = QGridLayout()
        servo_layout.setVerticalSpacing(8)
        servo_layout.setHorizontalSpacing(8)

        self.open_buttons = []
        self.close_buttons = []
        self.open_full_buttons = []

        for i in range(4):
            open_btn = QPushButton(f"🔼 50% (S{i+1})")
            close_btn = QPushButton(f"🔽 Close (S{i+1})")
            full_btn = QPushButton(f"🔼 100% (S{i+1})")
            
            for btn in [open_btn, close_btn, full_btn]:
                btn.setFont(QFont('Arial', 11))
                btn.setFixedHeight(40)
                btn.setMinimumWidth(120)
            
            open_btn.clicked.connect(lambda _, i=i: self.send_command(i, config.SERVO_OPEN_ANGLE))
            close_btn.clicked.connect(lambda _, i=i: self.send_command(i, config.SERVO_CLOSED_ANGLE))
            full_btn.clicked.connect(lambda _, i=i: self.send_command(i, config.SERVO_FULL_OPEN_ANGLE))
            
            servo_layout.addWidget(open_btn, i, 0)
            servo_layout.addWidget(close_btn, i, 1)
            servo_layout.addWidget(full_btn, i, 2)
            
            self.open_buttons.append(open_btn)
            self.close_buttons.append(close_btn)
            self.open_full_buttons.append(full_btn)

        servo_group.setLayout(servo_layout)
        right_column.layout().addWidget(servo_group)

        # Tools Control Group
        tools_group = QGroupBox("Tools Control")
        tools_group.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        tools_layout = QVBoxLayout()
        tools_layout.setSpacing(10)

        # Heater Control
        heater_group = QGroupBox("Heater")
        heater_group.setFont(QFont('Arial', 11, QFont.Weight.Bold))
        heater_btn_layout = QHBoxLayout()
        
        self.heater_on_button = QPushButton("🔥 ON")
        self.heater_off_button = QPushButton("❄ OFF")
        
        for btn in [self.heater_on_button, self.heater_off_button]:
            btn.setFont(QFont('Arial', 11))
            btn.setFixedHeight(40)
            btn.setMinimumWidth(180)
        
        self.heater_on_button.clicked.connect(lambda: self.send_heater_command(True))
        self.heater_off_button.clicked.connect(lambda: self.send_heater_command(False))
        
        heater_btn_layout.addWidget(self.heater_on_button)
        heater_btn_layout.addWidget(self.heater_off_button)
        heater_group.setLayout(heater_btn_layout)
        tools_layout.addWidget(heater_group)

        # Drill Control
        drill_group = QGroupBox("Drill")
        drill_group.setFont(QFont('Arial', 11, QFont.Weight.Bold))
        drill_btn_layout = QHBoxLayout()
        
        self.wiertlo_on_button = QPushButton("🔄 ON")
        self.wiertlo_off_button = QPushButton("⏹ OFF")
        
        for btn in [self.wiertlo_on_button, self.wiertlo_off_button]:
            btn.setFont(QFont('Arial', 11))
            btn.setFixedHeight(40)
            btn.setMinimumWidth(180)
        
        self.wiertlo_on_button.clicked.connect(lambda: self.send_wiertlo_command(True))
        self.wiertlo_off_button.clicked.connect(lambda: self.send_wiertlo_command(False))
        
        drill_btn_layout.addWidget(self.wiertlo_on_button)
        drill_btn_layout.addWidget(self.wiertlo_off_button)
        drill_group.setLayout(drill_btn_layout)
        tools_layout.addWidget(drill_group)

        tools_group.setLayout(tools_layout)
        right_column.layout().addWidget(tools_group)
        right_column.layout().addStretch()

        # Auto-close servos if configured
        if config.AUTO_CLOSE_SERVOS_ON_APP_START:
            self.close_all_servos()
        else:
            for index in range(4):
                self.update_servo_state(index, config.SERVO_CLOSED_ANGLE)

        # Add columns to main layout
        main_layout.addLayout(left_column, 1)  # Rozciągnięta lewa kolumna
        main_layout.addWidget(right_column)    # Stała szerokość prawej kolumny

        self.setLayout(main_layout)
        
        # Ustaw domyślne style dla przycisków narzędzi
        self.update_button_style(self.heater_off_button, config.BUTTON_OFF_COLOR)
        self.update_button_style(self.wiertlo_off_button, config.BUTTON_OFF_COLOR)
        self.update_button_style(self.heater_on_button, config.BUTTON_DEFAULT_COLOR)
        self.update_button_style(self.wiertlo_on_button, config.BUTTON_DEFAULT_COLOR)
        
        self.apply_styles()
        
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
            }
            QPushButton:hover {
                background-color: #555;
                border: 2px solid #666;
            }
            QLabel {
                padding: 5px;
            }
            QFrame {
                border-radius: 5px;
                background-color: #3a3a3a;
            }
        """)

        # Additional dynamic styles
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

    # ... (reszta metod pozostaje bez zmian)
    def init_ros_subscriptions(self):
        self.node.create_subscription(Float32MultiArray, '/temperature_data', self.temperature_callback, 10)
        self.node.create_subscription(Float32MultiArray, '/mass_data', self.mass_callback, 10)
        self.node.create_subscription(Float32MultiArray, '/soil_moisture_data', self.soil_moisture_callback, 10)

    def init_ros_publishers(self):
        self.servo_publishers = [
            self.node.create_publisher(Int32, f'/microros/servo{i+1}_topic', 10) for i in range(4)
        ]
        self.heater_publisher = self.node.create_publisher(Int8MultiArray, '/ESP32_GIZ/output_state_topic', 10)

    def send_heater_command(self, state: bool):
        self.heater_state = int(state)
        msg = Int8MultiArray()
        msg.data = [self.drill_state, self.heater_state, 0]
        self.heater_publisher.publish(msg)
        self.update_button_style(self.heater_on_button, config.BUTTON_ON_COLOR if state else config.BUTTON_DEFAULT_COLOR)
        self.update_button_style(self.heater_off_button, config.BUTTON_DEFAULT_COLOR if state else config.BUTTON_OFF_COLOR)

    def send_wiertlo_command(self, state: bool):
        self.drill_state = int(state)
        msg = Int8MultiArray()
        msg.data = [self.drill_state, self.heater_state, 0]
        self.heater_publisher.publish(msg)
        self.update_button_style(self.wiertlo_on_button, config.BUTTON_ON_COLOR if state else config.BUTTON_DEFAULT_COLOR)
        self.update_button_style(self.wiertlo_off_button, config.BUTTON_DEFAULT_COLOR if state else config.BUTTON_OFF_COLOR)

    def temperature_callback(self, msg: Float32MultiArray):
        self.time_steps += 1
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        for i, temperature in enumerate(msg.data):
            temperature = -temperature
            self.temperature_labels[i].setText(f'Sensor {i+1}:\n{temperature:.2f} °C')

            with open(f"{self.data_directory}/temperature_sensor_{i+1}.txt", "a") as file:
                file.write(f"{timestamp}, {temperature:.2f}\n")

    def mass_callback(self, msg: Float32MultiArray):
        self.time_steps += 1
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        for i, mass in enumerate(msg.data):
            self.mass_labels[i].setText(f'Sensor {i+1}:\n{mass:.2f} g')

            with open(f"{self.data_directory}/mass_sensor_{i+1}.txt", "a") as file:
                file.write(f"{timestamp}, {mass:.2f}\n")

    def soil_moisture_callback(self, msg: Float32MultiArray):
        self.time_steps += 1
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        for i, moisture in enumerate(msg.data):
            self.soil_moisture_labels[i].setText(f'Sensor {i+1}:\n{moisture:.2f}')

            with open(f"{self.data_directory}/soil_moisture_sensor_{i+1}.txt", "a") as file:
                file.write(f"{timestamp}, {moisture:.2f}\n")

    def send_command(self, index, value):
        self.update_servo_state(index, value)
        msg = Int32()
        msg.data = int(value)
        self.servo_publishers[index].publish(msg)

    def update_servo_state(self, index, value):
        self.servo_states[index] = value
        
        if value == config.SERVO_OPEN_ANGLE:
            self.update_button_style(self.open_buttons[index], config.BUTTON_ON_COLOR)
            self.update_button_style(self.close_buttons[index], config.BUTTON_DEFAULT_COLOR)
            self.update_button_style(self.open_full_buttons[index], config.BUTTON_DEFAULT_COLOR)
        elif value == config.SERVO_CLOSED_ANGLE:
            self.update_button_style(self.open_buttons[index], config.BUTTON_DEFAULT_COLOR)
            self.update_button_style(self.close_buttons[index], config.BUTTON_OFF_COLOR)
            self.update_button_style(self.open_full_buttons[index], config.BUTTON_DEFAULT_COLOR)
        elif value == config.SERVO_FULL_OPEN_ANGLE:
            self.update_button_style(self.open_buttons[index], config.BUTTON_DEFAULT_COLOR)
            self.update_button_style(self.close_buttons[index], config.BUTTON_DEFAULT_COLOR)
            self.update_button_style(self.open_full_buttons[index], config.BUTTON_ON_COLOR)

    def update_button_style(self, button, color):
        button.setStyleSheet(f"background-color: {color};")
        button.update()

    def close_all_servos(self):
        for button in self.close_buttons:
            button.click()

    def launch_plot_app(self):
        subprocess.Popen(["python3", "wykresy.py"])