from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, 
                            QGroupBox, QLineEdit, QFrame, QSpacerItem, QSizePolicy)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont, QPalette
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import threading
import pygame
import time
import keyboard

class ServaTab(QWidget):
    def __init__(self, node: Node, gamepads):
        super().__init__()
        self.node = node
        self.gamepads = gamepads
        self.selected_gamepad = None
        self.running = False
        self.is_gamepad_active = False
        
        self.servo_positions = [84, 90, 90, 90]
        self.step_values = [10, 10, 90, 90]
        self.first_servo = 84
        
        # Ograniczenia dla serwa 360°
        self.min_360_angle = -360
        self.max_360_angle = 360
        
        # Gamepad control variables
        self.last_gamepad_update = 0
        self.gamepad_update_interval = 0.2  # seconds between updates
        self.gamepad_buttons_pressed = {
            'axis_0': False,
            'axis_1': False,
            'axis_2': False,
            'axis_3': False
        }
        
        self.init_ui()
        self.init_ros_publisher()
        self.select_first_available_gamepad()
        '''
        LISTENING WHEN APP IS IN THE BACKGROUND
        '''
        #self.key_listener_thread = threading.Thread(target=self.listen_for_global_keys, daemon=True)
        #self.key_listener_thread.start()


    def init_ui(self):
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(20)

        # Dark theme styling
        self.setStyleSheet("""
            QWidget {
                background-color: #2D2D2D;
                color: #E0E0E0;
            }
            QGroupBox {
                font-size: 14px;
                font-weight: bold;
                border: 2px solid #444;
                border-radius: 8px;
                margin-top: 12px;
                padding-top: 8px;
                background-color: #333;
            }
            QLabel {
                font-size: 13px;
                color: #EEE;
            }
            QPushButton {
                font-size: 13px;
                min-height: 35px;
                padding: 6px;
                border-radius: 5px;
                background-color: #444;
                color: white;
            }
            QPushButton:hover {
                background-color: #555;
            }
            QLineEdit {
                font-size: 13px;
                padding: 6px;
                border-radius: 4px;
                border: 1px solid #555;
                background-color: #3A3A3A;
                color: white;
                selection-background-color: #555;
            }
            QFrame {
                border-radius: 6px;
                background-color: #3A3A3A;
                border: 1px solid #444;
            }
        """)

        # Servo control group
        self.servo_group = QGroupBox("SERVO CONTROL")
        servo_layout = QHBoxLayout()
        servo_layout.setSpacing(25)
        
        servo_names = [
            "360° Camera", 
            "180° Camera", 
            "JEDNO Z SERW 180", 
            "DRUGIE Z SERW 180"
        ]
        
        self.servo_labels = []
        self.step_inputs = []
        self.update_buttons = []
        self.increase_buttons = []
        self.decrease_buttons = []
        
        for i in range(4):
            # Frame for each servo
            frame = QFrame()
            frame.setFrameShape(QFrame.Shape.StyledPanel)
            frame.setStyleSheet("padding: 6px;")
            
            column_layout = QVBoxLayout(frame)
            column_layout.setSpacing(10)
            
            # Servo name label
            name_label = QLabel(f"<b>{servo_names[i]}</b>")
            name_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            name_label.setStyleSheet("font-size: 14px; color: #DDD;")
            column_layout.addWidget(name_label)
            
            # Current position
            pos_label = QLabel(f"Position: {self.servo_positions[i]}°")
            pos_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            pos_label.setStyleSheet("font-size: 14px; font-weight: bold; color: #3498DB;")
            self.servo_labels.append(pos_label)
            column_layout.addWidget(pos_label)
            
            # Step control
            step_layout = QHBoxLayout()
            step_layout.addWidget(QLabel("Step:"))
            self.step_inputs.append(QLineEdit(str(self.step_values[i])))
            self.step_inputs[i].setMaximumWidth(60)
            self.step_inputs[i].setAlignment(Qt.AlignmentFlag.AlignCenter)
            step_layout.addWidget(self.step_inputs[i])
            column_layout.addLayout(step_layout)
            
            # Update step button
            update_btn = QPushButton("Set Step")
            update_btn.setStyleSheet("""
                QPushButton {
                    background-color: #505050;
                    border: 1px solid #666;
                }
                QPushButton:hover {
                    background-color: #606060;
                }
            """)
            self.update_buttons.append(update_btn)
            column_layout.addWidget(update_btn)
            
            # Control buttons
            control_layout = QHBoxLayout()
            control_layout.setSpacing(10)
            
            decrease_btn = QPushButton("◄")
            decrease_btn.setStyleSheet("""
                QPushButton {
                    font-size: 18px; 
                    min-width: 50px;
                    background-color: #505050;
                    color: white;
                }
                QPushButton:hover {
                    background-color: #606060;
                }
            """)
            
            # Add reset button only for 360° Camera (index 0)
            if i == 0:
                reset_btn = QPushButton("●")
                reset_btn.setStyleSheet("""
                    QPushButton {
                        font-size: 18px; 
                        min-width: 50px;
                        background-color: #505050;
                        color: white;
                    }
                    QPushButton:hover {
                        background-color: #606060;
                    }
                """)
                reset_btn.clicked.connect(self.reset_360_servo)
            
            increase_btn = QPushButton("►")
            increase_btn.setStyleSheet("""
                QPushButton {
                    font-size: 18px; 
                    min-width: 50px;
                    background-color: #505050;
                    color: white;
                }
                QPushButton:hover {
                    background-color: #606060;
                }
            """)
            
            self.decrease_buttons.append(decrease_btn)
            self.increase_buttons.append(increase_btn)
            
            control_layout.addWidget(decrease_btn)
            if i == 0:
                control_layout.addWidget(reset_btn)
            control_layout.addWidget(increase_btn)
            column_layout.addLayout(control_layout)
            
            # Add frame to layout
            servo_layout.addWidget(frame)
            
            # Connect signals
            self.update_buttons[i].clicked.connect(lambda _, i=i: self.update_step_value(i))
            
            if i == 0:
                # Special handling for 360° servo
                self.increase_buttons[i].pressed.connect(lambda: self.adjust_servo_position(0, self.step_values[0]))
                self.increase_buttons[i].released.connect(self.reset_360_servo)
                self.decrease_buttons[i].pressed.connect(lambda: self.adjust_servo_position(0, -self.step_values[0]))
                self.decrease_buttons[i].released.connect(self.reset_360_servo)
            else:
                # Standard handling for other servos
                self.increase_buttons[i].clicked.connect(lambda _, i=i: self.adjust_servo_position(i, self.step_values[i]))
                self.decrease_buttons[i].clicked.connect(lambda _, i=i: self.adjust_servo_position(i, -self.step_values[i]))
        
        self.servo_group.setLayout(servo_layout)
        main_layout.addWidget(self.servo_group)

        # Gamepad control group
        gamepad_group = QGroupBox("EXTERNAL CONTROL")
        gamepad_layout = QVBoxLayout()
        gamepad_layout.setSpacing(15)
        
        self.gamepad_toggle_button = QPushButton("GAMEPAD: DISABLED")
        self.gamepad_toggle_button.setStyleSheet("""
            QPushButton {
                font-size: 16px;
                font-weight: bold;
                min-height: 50px;
                background-color: #505050;
                color: white;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #606060;
            }
        """)
        self.gamepad_toggle_button.clicked.connect(self.toggle_gamepad)
        
        # Bottom row with two frames
        bottom_row = QHBoxLayout()
        bottom_row.setSpacing(15)
        
        # Keyboard instruction frame
        instruction_frame = QFrame()
        instruction_frame.setFrameShape(QFrame.Shape.StyledPanel)
        instruction_frame.setStyleSheet("padding: 6px;")
        
        instruction_layout = QVBoxLayout(instruction_frame)
        instruction_label = QLabel('''
            <div style='font-size: 13px; color: #AAA;'>
            <b>KEYBOARD CONTROL:</b><br>
            <table width='100%'>
            <tr><td>360° Camera:</td><td>A (left) / D (right)</td></tr>
            <tr><td>180° Camera:</td><td>W (up) / S (down)</td></tr>
            <tr><td>Science Camera:</td><td>K (up) / L (down)</td></tr>
            <tr><td>Forever Alone Module:</td><td>U (up) / J (down)</td></tr>
            </table>
            </div>
        ''')
        instruction_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        instruction_layout.addWidget(instruction_label)
        
        # First servo position frame
        first_servo_frame = QFrame()
        first_servo_frame.setFrameShape(QFrame.Shape.StyledPanel)
        first_servo_frame.setStyleSheet("padding: 6px;")
        
        first_servo_layout = QVBoxLayout(first_servo_frame)
        first_servo_layout.setSpacing(10)
        
        first_servo_label = QLabel("<b>360° CAMERA SETTINGS</b>")
        first_servo_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        first_servo_layout.addWidget(first_servo_label)
        
        # Input for first servo position
        input_layout = QHBoxLayout()
        input_layout.addWidget(QLabel("Default position:"))
        self.first_servo_input = QLineEdit(str(self.first_servo))
        self.first_servo_input.setMaximumWidth(60)
        self.first_servo_input.setAlignment(Qt.AlignmentFlag.AlignCenter)
        input_layout.addWidget(self.first_servo_input)
        first_servo_layout.addLayout(input_layout)
        
        # Update button
        update_first_servo_btn = QPushButton("Update Position")
        update_first_servo_btn.setStyleSheet("""
            QPushButton {
                background-color: #505050;
                border: 1px solid #666;
            }
            QPushButton:hover {
                background-color: #606060;
            }
        """)
        update_first_servo_btn.clicked.connect(self.update_first_servo_position)
        first_servo_layout.addWidget(update_first_servo_btn)
        
        # Add frames to bottom row
        bottom_row.addWidget(instruction_frame)
        bottom_row.addWidget(first_servo_frame)
        
        gamepad_layout.addWidget(self.gamepad_toggle_button)
        gamepad_layout.addLayout(bottom_row)
        gamepad_group.setLayout(gamepad_layout)
        
        main_layout.addWidget(gamepad_group)
        main_layout.addItem(QSpacerItem(20, 20, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))
        
        self.setLayout(main_layout)

    def update_first_servo_position(self):
        """Update the first servo's default position"""
        try:
            new_value = int(self.first_servo_input.text())
            if self.min_360_angle <= new_value <= self.max_360_angle:
                self.first_servo = new_value
                self.servo_positions[0] = new_value
                self.servo_labels[0].setText(f"Position: {self.servo_positions[0]}°")
                self.publish_servo_positions()
            else:
                print(f"Position should be between {self.min_360_angle} and {self.max_360_angle}")
        except ValueError:
            print("Invalid position value. Please enter a valid integer.")

    def reset_360_servo(self):
        """Reset 360° servo to initial position"""
        self.servo_positions[0] = self.first_servo
        self.servo_labels[0].setText(f"Position: {self.servo_positions[0]}°")
        self.publish_servo_positions()

    def init_ros_publisher(self):
        self.servo_publisher = self.node.create_publisher(Int32MultiArray, '/ESP32_GIZ/servo_angles_topic', 10)

    def select_first_available_gamepad(self):
        if self.gamepads:
            self.set_selected_gamepad(self.gamepads[0])
    
    def set_selected_gamepad(self, gamepad):
        self.selected_gamepad = gamepad
        if self.selected_gamepad:
            self.running = True
            self.gamepad_thread = threading.Thread(target=self.read_gamepad, daemon=True)
            self.gamepad_thread.start()
        else:
            self.running = False
    
    def update_step_value(self, index):
        try:
            new_value = int(self.step_inputs[index].text())
            if 1 <= new_value <= 90:
                self.step_values[index] = new_value
            else:
                print("Step value should be between 1 and 30")
        except ValueError:
            print("Invalid step value. Please enter a valid integer.")
    
    def adjust_servo_position(self, index, delta):
        if index == 0:
            # Special handling for 360° servo
            new_position = self.servo_positions[index] + delta
            if new_position < self.min_360_angle:
                new_position = self.min_360_angle
            elif new_position > self.max_360_angle:
                new_position = self.max_360_angle
            self.servo_positions[index] = new_position
        else:
            # Standard handling for 180° servos
            new_position = self.servo_positions[index] + delta
            if new_position < 0:
                new_position = 0
            elif new_position > 180:
                new_position = 180
            self.servo_positions[index] = new_position
        
        self.servo_labels[index].setText(f"Position: {self.servo_positions[index]}°")
        self.publish_servo_positions()
    
    def publish_servo_positions(self):
        msg = Int32MultiArray()
        msg.data = self.servo_positions
        self.servo_publisher.publish(msg)
    
    def read_gamepad(self):
        dead_zone = 0.8
        clock = pygame.time.Clock()
        
        while self.running and self.selected_gamepad:
            if self.is_gamepad_active:
                pygame.event.pump()
                current_time = time.time()
                
                # Check axes only if enough time has passed since last update
                if current_time - self.last_gamepad_update >= self.gamepad_update_interval:
                    axis_0 = self.selected_gamepad.get_axis(3)  # Left stick X (360° servo)
                    axis_1 = self.selected_gamepad.get_axis(4)  # Right stick Y (180° camera)
                    
                    # Handle 360° servo (index 0)
                    if abs(axis_0) > dead_zone:
                        if not self.gamepad_buttons_pressed['axis_0']:
                            # Single position change
                            self.adjust_servo_position(0, int(axis_0 * self.step_values[0]))
                            self.gamepad_buttons_pressed['axis_0'] = True
                            self.last_gamepad_update = current_time
                    else:
                        if self.gamepad_buttons_pressed['axis_0']:
                            self.reset_360_servo()
                            self.gamepad_buttons_pressed['axis_0'] = False
                    
                    # Handle 180° servo (index 1)
                    if abs(axis_1) > dead_zone:
                        if not self.gamepad_buttons_pressed['axis_1']:
                            # Single position change
                            self.adjust_servo_position(1, int(axis_1 * self.step_values[1]))
                            self.gamepad_buttons_pressed['axis_1'] = True
                            self.last_gamepad_update = current_time
                    else:
                        self.gamepad_buttons_pressed['axis_1'] = False
                
                clock.tick(30)  # Limit to 30 FPS
            else:
                pygame.time.wait(100)
    
    def toggle_gamepad(self):
        self.is_gamepad_active = not self.is_gamepad_active
        if self.is_gamepad_active:
            self.gamepad_toggle_button.setText("GAMEPAD: ENABLED")
            self.gamepad_toggle_button.setStyleSheet("""
                QPushButton {
                    font-size: 16px;
                    font-weight: bold;
                    min-height: 50px;
                    background-color: #27AE60;
                    color: white;
                    border-radius: 8px;
                }
                QPushButton:hover {
                    background-color: #2ECC71;
                }
            """)
        else:
            self.gamepad_toggle_button.setText("GAMEPAD: DISABLED")
            self.gamepad_toggle_button.setStyleSheet("""
                QPushButton {
                    font-size: 16px;
                    font-weight: bold;
                    min-height: 50px;
                    background-color: #505050;
                    color: white;
                    border-radius: 8px;
                }
                QPushButton:hover {
                    background-color: #606060;
                }
            """)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_W:
            self.adjust_servo_position(1, self.step_values[1])  # 180° Camera - up
        elif event.key() == Qt.Key.Key_S:
            self.adjust_servo_position(1, -self.step_values[1])  # 180° Camera - down
        elif event.key() == Qt.Key.Key_A:
            self.adjust_servo_position(0, -self.step_values[0])  # 360° Camera - left
        elif event.key() == Qt.Key.Key_D:
            self.adjust_servo_position(0, self.step_values[0])  # 360° Camera - right
        elif event.key() == Qt.Key.Key_K:
            self.adjust_servo_position(2, self.step_values[2])  # Science Camera - up
        elif event.key() == Qt.Key.Key_L:
            self.adjust_servo_position(2, -self.step_values[2])  # Science Camera - down
        elif event.key() == Qt.Key.Key_U:
            self.adjust_servo_position(3, self.step_values[3])  # Forever Alone Module - up
        elif event.key() == Qt.Key.Key_J:
            self.adjust_servo_position(3, -self.step_values[3])  # Forever Alone Module - down

    def listen_for_global_keys(self):
        while True:
            if keyboard.is_pressed('w'):
                self.adjust_servo_position(1, self.step_values[1])  # 180° Camera - up
            elif keyboard.is_pressed('s'):
                self.adjust_servo_position(1, -self.step_values[1])  # 180° Camera - down
            elif keyboard.is_pressed('a'):
                self.adjust_servo_position(0, -self.step_values[0])  # 360° Camera - left
            elif keyboard.is_pressed('d'):
                self.adjust_servo_position(0, self.step_values[0])  # 360° Camera - right
            elif keyboard.is_pressed('k'):
                self.adjust_servo_position(2, self.step_values[2])  # Science Camera - up
            elif keyboard.is_pressed('l'):
                self.adjust_servo_position(2, -self.step_values[2])  # Science Camera - down
            elif keyboard.is_pressed('u'):
                self.adjust_servo_position(3, self.step_values[3])  # Forever Alone Module - up
            elif keyboard.is_pressed('j'):
                self.adjust_servo_position(3, -self.step_values[3])  # Forever Alone Module - down
            time.sleep(0.1)  # Sleep to prevent high CPU usage

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key.Key_A or event.key() == Qt.Key.Key_D:
            self.reset_360_servo()

    def closeEvent(self, event):
        self.running = False
        if hasattr(self, 'gamepad_thread') and self.gamepad_thread.is_alive():
            self.gamepad_thread.join()
        event.accept()