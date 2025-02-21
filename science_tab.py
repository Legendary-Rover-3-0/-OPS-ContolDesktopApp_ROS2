from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ScienceTab(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.init_ui()
        self.init_ros_subscriptions()

    def init_ui(self):
        layout = QVBoxLayout()

        self.temperature_labels = [QLabel(f'Temperatura czujnika {i+1}: Oczekiwanie na dane...') for i in range(4)]
        self.mass_labels = [QLabel(f'Masa czujnika {i+1}: Oczekiwanie na dane...') for i in range(4)]
        self.servo_labels = [QLabel(f'Pozycja serwa {i+1}: Oczekiwanie na dane...') for i in range(4)]
        self.soil_moisture_labels = [QLabel(f'Wilgotność gleby czujnika {i+1}: Oczekiwanie na dane...') for i in range(4)]

        for label in self.temperature_labels + self.mass_labels + self.servo_labels + self.soil_moisture_labels:
            layout.addWidget(label)

        self.setLayout(layout)

    def init_ros_subscriptions(self):
        self.node.create_subscription(Float32MultiArray, '/temperature_data', self.temperature_callback, 10)
        self.node.create_subscription(Float32MultiArray, '/mass_data', self.mass_callback, 10)
        self.node.create_subscription(Float32MultiArray, '/servo_position_data', self.servo_callback, 10)
        self.node.create_subscription(Float32MultiArray, '/soil_moisture_data', self.soil_moisture_callback, 10)

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
