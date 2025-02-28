from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox
from PyQt6.QtCore import Qt
from rclpy.node import Node
from std_msgs.msg import Int32

class ServaTab(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        

        # Initialize servo positions
        self.servo_positions = [90, 90, 90]  # Default position for each servo

        self.init_ui()
        self.init_ros_publishers()

    def init_ui(self):
        main_layout = QVBoxLayout()

        # Group box for servo control
        self.servo_group = QGroupBox("Servo Control")
        servo_layout = QVBoxLayout()

        # Labels to display current servo positions
        self.servo_labels = [QLabel(f'Servo {i+1} Position: {self.servo_positions[i]}°') for i in range(3)]
        for label in self.servo_labels:
            servo_layout.addWidget(label)

        # Buttons to increase and decrease servo positions
        self.buttons_layout = QHBoxLayout()
        self.increase_buttons = [QPushButton(f'Increase Servo {i+1}') for i in range(3)]
        self.decrease_buttons = [QPushButton(f'Decrease Servo {i+1}') for i in range(3)]

        for i, (increase_button, decrease_button) in enumerate(zip(self.increase_buttons, self.decrease_buttons)):
            self.buttons_layout.addWidget(increase_button)
            self.buttons_layout.addWidget(decrease_button)

            # Connect buttons to increase and decrease methods
            increase_button.clicked.connect(lambda _, i=i: self.adjust_servo_position(i, 5))
            decrease_button.clicked.connect(lambda _, i=i: self.adjust_servo_position(i, -5))

        servo_layout.addLayout(self.buttons_layout)
        self.servo_group.setLayout(servo_layout)

        main_layout.addWidget(self.servo_group)
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

    def init_ros_publishers(self):
        self.servo_publishers = [
            self.node.create_publisher(Int32, f'/servo{i+1}', 10) for i in range(3)
        ]

    def adjust_servo_position(self, index, delta):
        # Adjust the servo position by delta, ensuring it stays within 0-180 degrees
        self.servo_positions[index] = max(0, min(180, self.servo_positions[index] + delta))
        self.servo_labels[index].setText(f'Servo {index+1} Position: {self.servo_positions[index]}°')

        # Publish the new position
        msg = Int32()
        msg.data = self.servo_positions[index]
        self.servo_publishers[index].publish(msg)