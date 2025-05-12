from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox, QGridLayout, QFrame
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QPalette, QFont
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray, Int8MultiArray, Float32
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
        self.radiation_history = [[]]
        self.gas_history = [[] for _ in range(4)]
        self.pH_history = [[]]
        self.time_steps = 0
        self.update_interval = 5
        self.update_counter = 0

        # Initialize mass offsets for taring
        self.mass_offsets = [0.0] * 4

        # Create data directory
        self.data_directory = "sensor_data"
        if not os.path.exists(self.data_directory):
            os.makedirs(self.data_directory)

        # Servo states
        self.servo_states = [0] * 4
        self.drill_state = 0
        self.heater_state = 0
        self.koszelnik_state = 0

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
        self.tare_buttons = []
        self.untare_buttons = []
        for i in range(4):
            frame = QFrame()
            frame.setFrameShape(QFrame.Shape.StyledPanel)
            frame.setLineWidth(1)
            frame_layout = QHBoxLayout(frame)
            
            # Create container for label to center it vertically
            label_container = QWidget()
            label_layout = QVBoxLayout(label_container)
            label_layout.setContentsMargins(0, 0, 0, 0)
            
            label = QLabel(f"Sensor {i+1}:\n--- g")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setFont(QFont('Arial', 11))
            label_layout.addWidget(label, alignment=Qt.AlignmentFlag.AlignVCenter)
            self.mass_labels.append(label)
            
            # Add buttons container
            buttons_container = QWidget()
            buttons_layout = QHBoxLayout(buttons_container)
            buttons_layout.setContentsMargins(0, 0, 0, 0)
            buttons_layout.setSpacing(3)
            
            # Add tare button
            tare_btn = QPushButton("T")
            tare_btn.setFont(QFont('Arial', 9))
            tare_btn.setFixedSize(30, 30)
            tare_btn.setToolTip("Taruj wagę")
            tare_btn.clicked.connect(lambda _, i=i: self.tare_scale(i))
            self.tare_buttons.append(tare_btn)
            
            # Add untare button
            untare_btn = QPushButton("C")
            untare_btn.setFont(QFont('Arial', 9))
            untare_btn.setFixedSize(30, 30)
            untare_btn.setToolTip("Cofnij tarowanie")
            untare_btn.clicked.connect(lambda _, i=i: self.untare_scale(i))
            self.untare_buttons.append(untare_btn)
            
            buttons_layout.addWidget(tare_btn)
            buttons_layout.addWidget(untare_btn)
            
            frame_layout.addWidget(label_container, stretch=1)
            frame_layout.addWidget(buttons_container)
            frame_layout.setSpacing(5)
            
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

        #Radiation Group
        self.radiation_group = QGroupBox("☢️ Radiation Sensors")
        self.radiation_group.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        radiation_layout = QGridLayout()
        radiation_layout.setSpacing(10)
        self.radiation_labels = []
        for i in range(1):
            frame = QFrame()
            frame.setFrameShape(QFrame.Shape.StyledPanel)
            frame.setLineWidth(1)
            frame_layout = QVBoxLayout(frame)
            
            label = QLabel(f"Sensor {i+1}:\n---")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setFont(QFont('Arial', 11))
            self.radiation_labels.append(label)
            frame_layout.addWidget(label)
            
            radiation_layout.addWidget(frame, i//2, i%2)
        self.radiation_group.setLayout(radiation_layout)
        #left_column.addWidget(self.radiation_group)

        # Gases Group
        self.gases_group = QGroupBox("💨 Gases Sensors")
        self.gases_group.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        gases_layout = QGridLayout()
        gases_layout.setSpacing(10)
        self.gases_labels = []
        for i in range(4):
            frame = QFrame()
            frame.setFrameShape(QFrame.Shape.StyledPanel)
            frame.setLineWidth(1)
            frame_layout = QVBoxLayout(frame)
            
            
            #label = QLabel(f"Sensdskajdsaor {i+1}:\n---")
            
            if i == 0:
                label = QLabel("Propan (ppm)")
            if i == 1:
                label = QLabel("Butan (ppm)")
            if i == 2:
                label = QLabel("CO (ppm)")
            if i == 3:
                label = QLabel("-")
            
            
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setFont(QFont('Arial', 11))
            self.gases_labels.append(label)
            frame_layout.addWidget(label)
            
            gases_layout.addWidget(frame, i//2, i%2)
        self.gases_group.setLayout(gases_layout)
        left_column.addWidget(self.gases_group)


        #pH Group
        self.pH_group = QGroupBox("💧 pH Sensors")
        self.pH_group.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        pH_layout = QGridLayout()
        pH_layout.setSpacing(10)
        self.pH_labels = []
        for i in range(1):
            frame = QFrame()
            frame.setFrameShape(QFrame.Shape.StyledPanel)
            frame.setLineWidth(1)
            frame_layout = QVBoxLayout(frame)
            
            label = QLabel(f"Sensor {i+1}:\n---")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setFont(QFont('Arial', 11))
            self.pH_labels.append(label)
            frame_layout.addWidget(label)
            
            pH_layout.addWidget(frame, i//2, i%2)
        self.pH_group.setLayout(pH_layout)
        


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

        self.little_open_buttons = []
        self.open_buttons = []
        self.close_buttons = []
        self.open_full_buttons = []

        for i in range(4):
            little_open_btn = QPushButton(f"🔼 30% (S{i+1})")
            open_btn = QPushButton(f"🔼 50% (S{i+1})")
            close_btn = QPushButton(f"🔽 Close (S{i+1})")
            full_btn = QPushButton(f"🔼 100% (S{i+1})")
            
            for btn in [little_open_btn, open_btn, close_btn, full_btn]:
                btn.setFont(QFont('Arial', 11))
                btn.setFixedHeight(40)
                btn.setMinimumWidth(120)
            
            little_open_btn.clicked.connect(lambda _, i=i: self.send_command(i, config.SERVO_LITTLE_OPEN_ANGLE))
            open_btn.clicked.connect(lambda _, i=i: self.send_command(i, config.SERVO_OPEN_ANGLE))
            close_btn.clicked.connect(lambda _, i=i: self.send_command(i, config.SERVO_CLOSED_ANGLE))
            full_btn.clicked.connect(lambda _, i=i: self.send_command(i, config.SERVO_FULL_OPEN_ANGLE))
            
            servo_layout.addWidget(little_open_btn, i, 0)
            servo_layout.addWidget(open_btn, i, 1)
            servo_layout.addWidget(close_btn, i, 2)
            servo_layout.addWidget(full_btn, i, 3)
            
            self.little_open_buttons.append(little_open_btn)
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
        drill_group = QGroupBox("Pompka #1")
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

        # Remote Koszelnik Control
        koszelnik_group = QGroupBox("Zdalny Koszelnik")
        koszelnik_group.setFont(QFont('Arial', 11, QFont.Weight.Bold))
        koszelnik_btn_layout = QHBoxLayout()
        
        self.koszelnik_on_button = QPushButton("🛠️ ON")
        self.koszelnik_off_button = QPushButton("🍺 OFF")
        
        for btn in [self.koszelnik_on_button, self.koszelnik_off_button]:
            btn.setFont(QFont('Arial', 11))
            btn.setFixedHeight(40)
            btn.setMinimumWidth(180)
        
        self.koszelnik_on_button.clicked.connect(lambda: self.send_koszelnik_command(True))
        self.koszelnik_off_button.clicked.connect(lambda: self.send_koszelnik_command(False))
        
        koszelnik_btn_layout.addWidget(self.koszelnik_on_button)
        koszelnik_btn_layout.addWidget(self.koszelnik_off_button)
        koszelnik_group.setLayout(koszelnik_btn_layout)
        tools_layout.addWidget(koszelnik_group)

        tools_group.setLayout(tools_layout)
        right_column.layout().addWidget(tools_group)

        right_column.layout().addWidget(self.pH_group)
        right_column.layout().addWidget(self.radiation_group)

        right_column.layout().addStretch()

        # Auto-close servos if configured
        if config.AUTO_CLOSE_SERVOS_ON_APP_START:
            self.close_all_servos()
        else:
            for index in range(4):
                self.update_servo_state(index, config.SERVO_CLOSED_ANGLE)


        # Add columns to main layout
        main_layout.addLayout(left_column, 1)
        main_layout.addWidget(right_column)

        self.setLayout(main_layout)
        
        # Ustaw domyślne style dla przycisków narzędzi
        self.update_button_style(self.heater_off_button, config.BUTTON_OFF_COLOR)
        self.update_button_style(self.wiertlo_off_button, config.BUTTON_OFF_COLOR)
        self.update_button_style(self.heater_on_button, config.BUTTON_DEFAULT_COLOR)
        self.update_button_style(self.wiertlo_on_button, config.BUTTON_DEFAULT_COLOR)
            # Ustawiamy domyślny styl przycisków Koszelnika
        self.update_button_style(self.koszelnik_on_button, config.BUTTON_DEFAULT_COLOR)
        self.update_button_style(self.koszelnik_off_button, config.BUTTON_OFF_COLOR)
        
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
            QPushButton[text="T"] {
                background-color: #5D6D7E;
                font-size: 9px;
                padding: 2px;
                min-width: 20px;
                max-width: 30px;
                max-height: 30px;
            }
            QPushButton[text="T"]:hover {
                background-color: #6D7E8E;
            }
            QPushButton[text="C"] {
                background-color: #7D8E9E;
                font-size: 9px;
                padding: 2px;
                min-width: 20px;
                max-width: 30px;
                max-height: 30px;
            }
            QPushButton[text="C"]:hover {
                background-color: #8D9EAE;
            }
            QToolTip {
                background-color: #454545;
                color: white;
                border: 1px solid #666;
                padding: 2px;
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

    def init_ros_subscriptions(self):
        self.node.create_subscription(Float32MultiArray, '/temperature_data', self.temperature_callback, 10)
        self.node.create_subscription(Float32MultiArray, '/mass_data', self.mass_callback, 10)
        self.node.create_subscription(Float32MultiArray, '/soil_moisture_data', self.soil_moisture_callback, 10)
        self.node.create_subscription(Float32, '/Radiation_Publisher', self.Radiation_Publisher_callback, 10)
        self.node.create_subscription(Float32MultiArray, '/Gases_Publisher', self.Gases_Publisher_callback, 10)
        #################################################
        #################################################
        #################################################
        self.node.create_subscription(Float32, '/ph_sensor', self.ph_callback, 10) 

    def init_ros_publishers(self):
        self.servo_publishers = [
            self.node.create_publisher(Int32, f'/microros/servo{i+1}_topic', 10) for i in range(4)
        ]
        self.heater_publisher = self.node.create_publisher(Int8MultiArray, '/ESP32_GIZ/output_state_topic', 10)

    # Modyfikujemy metody wysyłania komend, aby nie resetowały stanu Koszelnika:

    def send_heater_command(self, state: bool):
        self.heater_state = int(state)
        msg = Int8MultiArray()
        msg.data = [self.drill_state, self.koszelnik_state, self.heater_state]  # Zachowaj obecny stan Koszelnika
        self.heater_publisher.publish(msg)
        self.update_button_style(self.heater_on_button, config.BUTTON_ON_COLOR if state else config.BUTTON_DEFAULT_COLOR)
        self.update_button_style(self.heater_off_button, config.BUTTON_DEFAULT_COLOR if state else config.BUTTON_OFF_COLOR)

    def send_wiertlo_command(self, state: bool):
        self.drill_state = int(state)
        msg = Int8MultiArray()
        msg.data = [self.drill_state, self.koszelnik_state, self.heater_state]  # Zachowaj obecny stan Koszelnika
        self.heater_publisher.publish(msg)
        self.update_button_style(self.wiertlo_on_button, config.BUTTON_ON_COLOR if state else config.BUTTON_DEFAULT_COLOR)
        self.update_button_style(self.wiertlo_off_button, config.BUTTON_DEFAULT_COLOR if state else config.BUTTON_OFF_COLOR)

    def send_koszelnik_command(self, state: bool):
        self.koszelnik_state = int(state)
        msg = Int8MultiArray()
        msg.data = [self.drill_state, self.koszelnik_state, self.heater_state]
        self.heater_publisher.publish(msg)
        self.update_button_style(self.koszelnik_on_button, config.BUTTON_ON_COLOR if state else config.BUTTON_DEFAULT_COLOR)
        self.update_button_style(self.koszelnik_off_button, config.BUTTON_DEFAULT_COLOR if state else config.BUTTON_OFF_COLOR)

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
            corrected_mass = mass - self.mass_offsets[i]
            self.mass_labels[i].setText(f'Sensor {i+1}:\n{corrected_mass:.2f} g')

            with open(f"{self.data_directory}/mass_sensor_{i+1}.txt", "a") as file:
                file.write(f"{timestamp}, {corrected_mass:.2f}\n")

    def soil_moisture_callback(self, msg: Float32MultiArray):
        self.time_steps += 1
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        for i, moisture in enumerate(msg.data):
            self.soil_moisture_labels[i].setText(f'Sensor {i+1}:\n{moisture:.2f}')

            with open(f"{self.data_directory}/soil_moisture_sensor_{i+1}.txt", "a") as file:
                file.write(f"{timestamp}, {moisture:.2f}\n")

    def Radiation_Publisher_callback(self, msg: Float32):
        self.time_steps += 1
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.radiation_labels[0].setText(f'Sensor 1:\n{msg.data:.2f}')

        with open(f"{self.data_directory}/radiation_sensor.txt", "a") as file:
            file.write(f"{timestamp}, {msg.data:.2f}\n")

    def Gases_Publisher_callback(self, msg: Float32MultiArray):
        self.time_steps += 1
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        for i, gas in enumerate(msg.data):
            self.gases_labels[i].setText(f'Sensor {i+1}:\n{gas:.2f}')

            with open(f"{self.data_directory}/gas_sensor_{i+1}.txt", "a") as file:
                file.write(f"{timestamp}, {gas:.2f}\n")

    def ph_callback(self, msg: Float32):
        self.time_steps += 1
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.pH_labels[0].setText(f'Sensor 1:\n{msg.data:.2f}')

        with open(f"{self.data_directory}/ph_sensor.txt", "a") as file:
            file.write(f"{timestamp}, {msg.data:.2f}\n")

    def tare_scale(self, index):
        """Set current mass reading as zero point"""
        current_text = self.mass_labels[index].text()
        try:
            # Extract current displayed value
            current_value = float(current_text.split('\n')[1].replace(' g', ''))
            # Update offset so current value becomes zero
            self.mass_offsets[index] += current_value
            # Log the taring operation
            self.node.get_logger().info(f"Tared scale {index+1}, new offset: {self.mass_offsets[index]:.2f} g")
        except (IndexError, ValueError) as e:
            self.node.get_logger().warn(f"Could not tare scale {index+1}: {str(e)}")

    def untare_scale(self, index):
        """Reset the taring offset for the specified scale"""
        self.mass_offsets[index] = 0.0
        self.node.get_logger().info(f"Reset taring for scale {index+1}, offset set to 0")
        
        # Odśwież wyświetlaną wartość
        current_text = self.mass_labels[index].text()
        try:
            # Extract current displayed value
            current_value = float(current_text.split('\n')[1].replace(' g', ''))
            # Update display with raw value (current_value + old offset)
            raw_value = current_value + self.mass_offsets[index]
            self.mass_labels[index].setText(f'Sensor {index+1}:\n{raw_value:.2f} g')
        except (IndexError, ValueError) as e:
            self.node.get_logger().warn(f"Could not update scale {index+1} display: {str(e)}")

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
            self.update_button_style(self.little_open_buttons[index], config.BUTTON_DEFAULT_COLOR)
        elif value == config.SERVO_CLOSED_ANGLE:
            self.update_button_style(self.open_buttons[index], config.BUTTON_DEFAULT_COLOR)
            self.update_button_style(self.close_buttons[index], config.BUTTON_OFF_COLOR)
            self.update_button_style(self.open_full_buttons[index], config.BUTTON_DEFAULT_COLOR)
            self.update_button_style(self.little_open_buttons[index], config.BUTTON_DEFAULT_COLOR)
        elif value == config.SERVO_FULL_OPEN_ANGLE:
            self.update_button_style(self.open_buttons[index], config.BUTTON_DEFAULT_COLOR)
            self.update_button_style(self.close_buttons[index], config.BUTTON_DEFAULT_COLOR)
            self.update_button_style(self.open_full_buttons[index], config.BUTTON_ON_COLOR)
            self.update_button_style(self.little_open_buttons[index], config.BUTTON_DEFAULT_COLOR)
        elif value == config.SERVO_LITTLE_OPEN_ANGLE:
            self.update_button_style(self.open_buttons[index], config.BUTTON_DEFAULT_COLOR)
            self.update_button_style(self.close_buttons[index], config.BUTTON_DEFAULT_COLOR)
            self.update_button_style(self.open_full_buttons[index], config.BUTTON_DEFAULT_COLOR)
            self.update_button_style(self.little_open_buttons[index], config.BUTTON_ON_COLOR)

    def update_button_style(self, button, color):
        button.setStyleSheet(f"background-color: {color};")
        button.update()

    def close_all_servos(self):
        for button in self.close_buttons:
            button.click()

    def launch_plot_app(self):
        subprocess.Popen(["python3", "wykresy.py"])