from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox, QGridLayout
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QPalette
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray, Int8MultiArray
import os
import datetime
import config
from PyQt6.QtWidgets import QApplication  # Dodaj ten import na początku pliku
import subprocess


class ScienceTab(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        
        self.init_ros_subscriptions()
        self.init_ros_publishers()

        # Initialize data history for each sensor type
        self.max_data_points = 100  # Limit history to last 100 points
        self.temperature_history = [[] for _ in range(4)]  # 4 sensors
        self.mass_history = [[] for _ in range(4)]  # 4 sensors
        self.soil_moisture_history = [[] for _ in range(4)]  # 4 sensors
        self.time_steps = 0  # Counter for time steps

        # Variables for plot optimization
        self.update_interval = 5  # Update plots every 5 new data points
        self.update_counter = 0

        # Create a directory for saving data files
        self.data_directory = "sensor_data"
        if not os.path.exists(self.data_directory):
            os.makedirs(self.data_directory)

        # Servo states list, default all closed.
        self.servo_states = [0] * 4
        self.drill_state = 0
        self.heater_state = 0

        self.init_ui()

    def init_ui(self):
        main_layout = QHBoxLayout()

        # Left column for sensor data
        left_column = QVBoxLayout()

        # Group boxes for different sensor data
        self.temperature_group = QGroupBox("Temperature Data")
        self.mass_group = QGroupBox("Mass Data")
        self.soil_moisture_group = QGroupBox("Soil Moisture Data")

        self.temperature_layout = QGridLayout()
        self.mass_layout = QGridLayout()
        self.soil_moisture_layout = QGridLayout()

        self.temperature_labels = [QLabel(f'Sensor {i+1} Temperature: Waiting for data...') for i in range(4)]
        self.mass_labels = [QLabel(f'Sensor {i+1} Mass: Waiting for data...') for i in range(4)]
        self.soil_moisture_labels = [QLabel(f'Sensor {i+1} Soil Moisture: Waiting for data...') for i in range(4)]

        for i, label in enumerate(self.temperature_labels):
            self.temperature_layout.addWidget(label, i // 2, i % 2)
        for i, label in enumerate(self.mass_labels):
            self.mass_layout.addWidget(label, i // 2, i % 2)
        for i, label in enumerate(self.soil_moisture_labels):
            self.soil_moisture_layout.addWidget(label, i // 2, i % 2)

        self.temperature_group.setLayout(self.temperature_layout)
        self.mass_group.setLayout(self.mass_layout)
        self.soil_moisture_group.setLayout(self.soil_moisture_layout)

        # Dodaj przycisk do uruchamiania aplikacji z wykresami
        self.plot_app_button = QPushButton('Open Plot App')
        self.plot_app_button.clicked.connect(self.launch_plot_app)
        left_column.addWidget(self.plot_app_button)

        left_column.addWidget(self.temperature_group)
        left_column.addWidget(self.mass_group)
        left_column.addWidget(self.soil_moisture_group)

        # Add servo control buttons
        self.buttons_layout = QGridLayout()
        self.open_buttons = [QPushButton(f'Open Servo {i+1} (50%)') for i in range(4)]
        self.close_buttons = [QPushButton(f'Close Servo {i+1}') for i in range(4)]
        self.open_full_buttons = [QPushButton(f'Open Servo {i+1} (100%)') for i in range(4)]

        for i, (open_button, close_button, open_full_button) in enumerate(zip(self.open_buttons, self.close_buttons, self.open_full_buttons)):
            self.buttons_layout.addWidget(open_button, i, 0)
            self.buttons_layout.addWidget(close_button, i, 1)
            self.buttons_layout.addWidget(open_full_button, i, 2)

        left_column.addLayout(self.buttons_layout)

        # Connect buttons to send_command method
        for i, open_button in enumerate(self.open_buttons):
            open_button.clicked.connect(lambda _, i=i: self.send_command(i, config.SERVO_OPEN_ANGLE))  # 50% open (90 degrees)

        for i, close_button in enumerate(self.close_buttons):
            close_button.clicked.connect(lambda _, i=i: self.send_command(i, config.SERVO_CLOSED_ANGLE))  # Close (0 degrees)

        for i, open_full_button in enumerate(self.open_full_buttons):
            open_full_button.clicked.connect(lambda _, i=i: self.send_command(i, config.SERVO_FULL_OPEN_ANGLE))  # 100% open (180 degrees)

        # zamknięcie servo przy uruchomieniu aplikacji
        if config.AUTO_CLOSE_SERVOS_ON_APP_START:
            self.close_all_servos()
        else:
            for index, _ in enumerate(self.close_buttons):
                self.update_servo_state(index, config.SERVO_CLOSED_ANGLE)

        # Add heater control buttons
        self.heater_control_buttons = QHBoxLayout()
        self.heater_on_button = QPushButton('Turn Heater On')
        self.heater_off_button = QPushButton('Turn Heater Off')

        self.heater_on_button.clicked.connect(lambda: self.send_heater_command(True))
        self.heater_off_button.clicked.connect(lambda: self.send_heater_command(False))
        self.update_button_style(self.heater_off_button, config.BUTTON_OFF_COLOR)

        self.heater_control_buttons.addWidget(self.heater_on_button)
        self.heater_control_buttons.addWidget(self.heater_off_button)

        # Add wiertło buttons
        self.wiertlo_buttons = QHBoxLayout()
        self.wiertlo_on_button = QPushButton('Wiertło On')
        self.wiertlo_off_button = QPushButton('Wiertło Off')

        self.wiertlo_on_button.clicked.connect(lambda: self.send_wiertlo_command(True))
        self.wiertlo_off_button.clicked.connect(lambda: self.send_wiertlo_command(False))
        self.update_button_style(self.wiertlo_off_button, config.BUTTON_OFF_COLOR)

        self.wiertlo_buttons.addWidget(self.wiertlo_on_button)
        self.wiertlo_buttons.addWidget(self.wiertlo_off_button)

        left_column.addLayout(self.heater_control_buttons)  # Add heater control buttons
        left_column.addLayout(self.wiertlo_buttons)  # Add wiertło control buttons

        main_layout.addLayout(left_column, 1)

        self.setLayout(main_layout)

        # Setting some basic styling
        self.setStyleSheet("""
            QLabel {
                font-size: 12px;
                color: #ddd;
            }
            QPushButton {
                font-size: 10px;
                padding: 3px;
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

    def init_ros_subscriptions(self):
        self.node.create_subscription(Float32MultiArray, '/temperature_data', self.temperature_callback, 10)
        self.node.create_subscription(Float32MultiArray, '/mass_data', self.mass_callback, 10)
        self.node.create_subscription(Float32MultiArray, '/soil_moisture_data', self.soil_moisture_callback, 10)

    def init_ros_publishers(self):
        self.servo_publishers = [
            self.node.create_publisher(Int32, f'/microros/servo{i+1}_topic', 10) for i in range(4)
        ]
        # Change publisher to Int8MultiArray for output_state_topic
        self.heater_publisher = self.node.create_publisher(Int8MultiArray, '/ESP32_GIZ/output_state_topic', 10)

    def send_heater_command(self, state: bool):
        self.heater_state = int(state)
        msg = Int8MultiArray()
        msg.data = [self.drill_state, self.heater_state, 0]  # First state: heater, second state: wiertło, third state: reserved
        self.heater_publisher.publish(msg)
        self.update_button_style(self.heater_on_button, config.BUTTON_ON_COLOR if state else config.BUTTON_DEFAULT_COLOR)
        self.update_button_style(self.heater_off_button, config.BUTTON_DEFAULT_COLOR if state else config.BUTTON_OFF_COLOR)

    def send_wiertlo_command(self, state: bool):
        self.drill_state = int(state)
        msg = Int8MultiArray()
        msg.data = [self.drill_state, self.heater_state, 0]  # First state: heater, second state: wiertło, third state: reserved
        self.heater_publisher.publish(msg)
        self.update_button_style(self.wiertlo_on_button, config.BUTTON_ON_COLOR if state else config.BUTTON_DEFAULT_COLOR)
        self.update_button_style(self.wiertlo_off_button, config.BUTTON_DEFAULT_COLOR if state else config.BUTTON_OFF_COLOR)

    def temperature_callback(self, msg: Float32MultiArray):
        self.time_steps += 1
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        for i, temperature in enumerate(msg.data):
            temperature = -temperature
            self.temperature_labels[i].setText(f'Sensor {i+1} Temperature: \n {temperature:.2f} °C')

            # Save temperature data to file
            with open(f"{self.data_directory}/temperature_sensor_{i+1}.txt", "a") as file:
                file.write(f"{timestamp}, {temperature:.2f}\n")

    def mass_callback(self, msg: Float32MultiArray):
        self.time_steps += 1
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        for i, mass in enumerate(msg.data):
            self.mass_labels[i].setText(f'Sensor {i+1} Mass: \n{mass:.2f} g')

            # Save mass data to file
            with open(f"{self.data_directory}/mass_sensor_{i+1}.txt", "a") as file:
                file.write(f"{timestamp}, {mass:.2f}\n")

    def soil_moisture_callback(self, msg: Float32MultiArray):
        self.time_steps += 1
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        for i, moisture in enumerate(msg.data):
            self.soil_moisture_labels[i].setText(f'Sensor {i+1} Soil Moisture: \n{moisture:.2f}')

            # Save soil moisture data to file
            with open(f"{self.data_directory}/soil_moisture_sensor_{i+1}.txt", "a") as file:
                file.write(f"{timestamp}, {moisture:.2f}\n")

    def send_command(self, index, value):
        self.update_servo_state(index, value)
        msg = Int32()
        msg.data = int(value)
        self.servo_publishers[index].publish(msg)

    def update_servo_state(self, index, value):
        # update states list
        self.servo_states[index] = value
        
        # update buttons
        if value == config.SERVO_OPEN_ANGLE: #50% - 90
            self.update_button_style(self.open_buttons[index], config.BUTTON_ON_COLOR)
            self.update_button_style(self.close_buttons[index], config.BUTTON_DEFAULT_COLOR)
            self.update_button_style(self.open_full_buttons[index], config.BUTTON_DEFAULT_COLOR)
        elif value == config.SERVO_CLOSED_ANGLE: # 0% - 0
            self.update_button_style(self.open_buttons[index], config.BUTTON_DEFAULT_COLOR)
            self.update_button_style(self.close_buttons[index], config.BUTTON_OFF_COLOR)
            self.update_button_style(self.open_full_buttons[index], config.BUTTON_DEFAULT_COLOR)
        elif value == config.SERVO_FULL_OPEN_ANGLE: #100% - 180
            self.update_button_style(self.open_buttons[index], config.BUTTON_DEFAULT_COLOR)
            self.update_button_style(self.close_buttons[index], config.BUTTON_DEFAULT_COLOR)
            self.update_button_style(self.open_full_buttons[index], config.BUTTON_ON_COLOR)

    def update_button_style(self, button, color):
        button.setStyleSheet(f"background-color: {color}")
        button.update()

    def close_all_servos(self):
        for button in self.close_buttons:
            button.click()

    def launch_plot_app(self):
        subprocess.Popen(["python3", "wykresy.py"])  # Uruchom nową aplikację