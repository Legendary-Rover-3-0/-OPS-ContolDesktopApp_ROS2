from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox, QGridLayout
from PyQt6.QtCore import Qt
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ScienceTab(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.init_ui()
        self.init_ros_subscriptions()
        self.init_ros_publisher()

    def init_ui(self):
        layout = QVBoxLayout()

        # Group boxes for different sensor data
        self.temperature_group = QGroupBox("Dane Temperatura")
        self.mass_group = QGroupBox("Dane Masa")
        self.servo_group = QGroupBox("Dane Serwo")
        self.soil_moisture_group = QGroupBox("Dane Wilgotność Gleby")

        self.temperature_layout = QGridLayout()
        self.mass_layout = QGridLayout()
        self.servo_layout = QGridLayout()
        self.soil_moisture_layout = QGridLayout()

        self.temperature_labels = [QLabel(f'Temperatura czujnika {i+1}: Oczekiwanie na dane...') for i in range(4)]
        self.mass_labels = [QLabel(f'Masa czujnika {i+1}: Oczekiwanie na dane...') for i in range(4)]
        self.servo_labels = [QLabel(f'Pozycja serwa {i+1}: Oczekiwanie na dane...') for i in range(4)]
        self.soil_moisture_labels = [QLabel(f'Wilgotność gleby czujnika {i+1}: Oczekiwanie na dane...') for i in range(4)]

        for i, label in enumerate(self.temperature_labels):
            self.temperature_layout.addWidget(label, i // 2, i % 2)
        for i, label in enumerate(self.mass_labels):
            self.mass_layout.addWidget(label, i // 2, i % 2)
        for i, label in enumerate(self.servo_labels):
            self.servo_layout.addWidget(label, i // 2, i % 2)
        for i, label in enumerate(self.soil_moisture_labels):
            self.soil_moisture_layout.addWidget(label, i // 2, i % 2)

        self.temperature_group.setLayout(self.temperature_layout)
        self.mass_group.setLayout(self.mass_layout)
        self.servo_group.setLayout(self.servo_layout)
        self.soil_moisture_group.setLayout(self.soil_moisture_layout)

        layout.addWidget(self.temperature_group)
        layout.addWidget(self.mass_group)
        layout.addWidget(self.servo_group)
        layout.addWidget(self.soil_moisture_group)

        self.buttons_layout = QGridLayout()
        self.open_buttons = [QPushButton(f'Otwórz serwo {i+1}') for i in range(4)]
        self.close_buttons = [QPushButton(f'Zamknij serwo {i+1}') for i in range(4)]

        for i, (open_button, close_button) in enumerate(zip(self.open_buttons, self.close_buttons)):
            self.buttons_layout.addWidget(open_button, i, 0)
            self.buttons_layout.addWidget(close_button, i, 1)

        layout.addLayout(self.buttons_layout)

        for i, open_button in enumerate(self.open_buttons):
            open_button.clicked.connect(lambda _, i=i: self.send_command(i, 90.0))

        for i, close_button in enumerate(self.close_buttons):
            close_button.clicked.connect(lambda _, i=i: self.send_command(i, 0.0))

        self.setLayout(layout)

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
        self.node.create_subscription(Float32MultiArray, '/servo_position_data', self.servo_callback, 10)
        self.node.create_subscription(Float32MultiArray, '/soil_moisture_data', self.soil_moisture_callback, 10)

    def init_ros_publisher(self):
        self.servo_publisher = self.node.create_publisher(Float32MultiArray, '/servo_positions', 10)
        self.servo_positions = [0.0] * 4  # Inicjalizacja zmiennych do przechowywania stanów serw

    def temperature_callback(self, msg: Float32MultiArray):
        for i, temperature in enumerate(msg.data):
            self.temperature_labels[i].setText(f'Temperatura czujnika {i+1}: {temperature:.2f} °C')

    def mass_callback(self, msg: Float32MultiArray):
        for i, mass in enumerate(msg.data):
            self.mass_labels[i].setText(f'Masa czujnika {i+1}: {mass:.2f} g')

    def servo_callback(self, msg: Float32MultiArray):
        for i, position in enumerate(msg.data):
            self.servo_labels[i].setText(f'Pozycja serwa {i+1}: {position:.2f} °')

    def soil_moisture_callback(self, msg: Float32MultiArray):
        for i, moisture in enumerate(msg.data):
            self.soil_moisture_labels[i].setText(f'Wilgotność gleby czujnika {i+1}: {moisture:.2f} %')

    def send_command(self, index, value):
        self.servo_positions[index] = value  # Aktualizacja odpowiedniej zmiennej
        msg = Float32MultiArray()
        msg.data = self.servo_positions  # Publikowanie całej tablicy
        self.servo_publisher.publish(msg)
