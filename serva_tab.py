from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox, QCheckBox
from PyQt6.QtCore import Qt
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import threading
import pygame

class ServaTab(QWidget):
    def __init__(self, node: Node, gamepads):
        super().__init__()
        self.node = node
        self.gamepads = gamepads
        self.selected_gamepad = None
        self.running = False

        # Initialize servo positions (4 servos, range 0-180)
        self.servo_positions = [90, 90, 90, 90]  # Default positions for each servo

        # Flag to control whether to use gamepad for servos 1 and 2
        self.use_gamepad_for_servos = False

        self.init_ui()
        self.init_ros_publisher()

    def init_ui(self):
        main_layout = QVBoxLayout()

        # Group box for servo control
        self.servo_group = QGroupBox("Servo Control")
        servo_layout = QVBoxLayout()

        # Checkbox to toggle gamepad control for servos 1 and 2
        self.gamepad_toggle = QCheckBox("Use Gamepad for Servos 1 and 2")
        self.gamepad_toggle.stateChanged.connect(self.toggle_gamepad_control)
        servo_layout.addWidget(self.gamepad_toggle)

        # Labels to display current servo positions
        self.servo_labels = [QLabel(f'Servo {i+1} Position: {self.servo_positions[i]}°') for i in range(4)]
        for label in self.servo_labels:
            servo_layout.addWidget(label)

        # Buttons to increase and decrease servo positions
        self.buttons_layout = QHBoxLayout()
        self.increase_buttons = [QPushButton(f'Increase Servo {i+1}') for i in range(4)]
        self.decrease_buttons = [QPushButton(f'Decrease Servo {i+1}') for i in range(4)]

        for i, (increase_button, decrease_button) in enumerate(zip(self.increase_buttons, self.decrease_buttons)):
            self.buttons_layout.addWidget(increase_button)
            self.buttons_layout.addWidget(decrease_button)

            # Connect buttons to increase and decrease methods
            increase_button.clicked.connect(lambda _, i=i: self.adjust_servo_position(i, 10))
            decrease_button.clicked.connect(lambda _, i=i: self.adjust_servo_position(i, -10))

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

    def init_ros_publisher(self):
        # Create a single publisher for all servo angles
        self.servo_publisher = self.node.create_publisher(Int32MultiArray, '/ESP32_GIZ/servo_angles_topic', 10)

    def set_selected_gamepad(self, gamepad):
        self.selected_gamepad = gamepad
        if self.selected_gamepad:
            self.running = True
            self.gamepad_thread = threading.Thread(target=self.read_gamepad, daemon=True)
            self.gamepad_thread.start()
        else:
            self.running = False

    def toggle_gamepad_control(self, state):
        # Toggle whether to use gamepad for servos 1 and 2
        self.use_gamepad_for_servos = state == Qt.CheckState.Checked.value

        # Reset servos 1 and 2 to 90° when toggling
        if self.use_gamepad_for_servos:
            self.servo_positions[0] = 90
            self.servo_positions[1] = 90
            self.servo_labels[0].setText(f'Servo 1 Position: {self.servo_positions[0]}°')
            self.servo_labels[1].setText(f'Servo 2 Position: {self.servo_positions[1]}°')
            self.publish_servo_positions()

    def adjust_servo_position(self, index, delta):
        # Adjust the servo position by delta, ensuring it stays within 0-180 degrees
        if not self.use_gamepad_for_servos or index >= 2:  # Allow button control for servos 3 and 4, or all if gamepad is off
            self.servo_positions[index] = max(0, min(180, self.servo_positions[index] + delta))
            self.servo_labels[index].setText(f'Servo {index+1} Position: {self.servo_positions[index]}°')

            # Publish all servo positions
            self.publish_servo_positions()

    def publish_servo_positions(self):
        # Publish all servo positions as Int32MultiArray
        msg = Int32MultiArray()
        msg.data = self.servo_positions
        self.servo_publisher.publish(msg)

    def read_gamepad(self):
        while self.running and self.selected_gamepad:
            pygame.event.pump()

            if self.use_gamepad_for_servos:
                # Read axes 0 and 1 for servo 1 and 2
                axis_0 = self.selected_gamepad.get_axis(0)  # Axis 0 (left stick X)
                axis_1 = self.selected_gamepad.get_axis(1)  # Axis 1 (left stick Y)

                # Map axes to servo positions (range 0-180)
                self.servo_positions[0] = int((axis_0 + 1) * 90)  # Map from [-1, 1] to [0, 180]
                self.servo_positions[1] = int((axis_1 + 1) * 90)  # Map from [-1, 1] to [0, 180]

                # Update labels
                self.servo_labels[0].setText(f'Servo 1 Position: {self.servo_positions[0]}°')
                self.servo_labels[1].setText(f'Servo 2 Position: {self.servo_positions[1]}°')

                # Publish all servo positions
                self.publish_servo_positions()

            # Add a small delay to avoid overloading the loop
            pygame.time.wait(50)

    def closeEvent(self, event):
        # Stop the gamepad thread when the window is closed
        self.running = False
        if hasattr(self, 'gamepad_thread') and self.gamepad_thread.is_alive():
            self.gamepad_thread.join()
        event.accept()