from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
                            QGroupBox, QLineEdit, QFrame, QSpacerItem, QSizePolicy)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont, QPalette
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import threading
import pygame
import time
# import keyboard # Keep commented out unless needed for global listening

class ServaTab(QWidget):
    def __init__(self, node: Node, gamepads):
        super().__init__()
        self.node = node
        self.gamepads = gamepads
        self.selected_gamepad = None
        self.running = False
        self.is_gamepad_active = False

        # Initial servo positions - these will be the *default* positions for 180 servos,
        # and the *initial* default for the 360 servos.
        self.servo_positions = [84, 90, 0, 0]

        # Store default positions for servos that reset on button release
        self.default_servo_0_pos = self.servo_positions[0] # Default for 360 Camera
        #self.default_servo_3_pos = self.servo_positions[3] # Default for 360 Servo 2
        #self.default_servo_2_pos = self.servo_positions[2]
        self.step_values = [10, 10, 180, 180]

        # Ograniczenia dla serwa 360° (these limit the *commanded* angle,
        # assuming the servo controller handles continuous rotation based on this)
        self.min_360_angle = -360
        self.max_360_angle = 360
        # Ograniczenia dla serwa 180°
        self.min_180_angle = 0
        self.max_180_angle = 180

        # Gamepad control variables
        self.last_gamepad_update = 0
        self.gamepad_update_interval = 0.1  # seconds between updates (increased frequency)
        self.gamepad_axis_dead_zone = 0.8 # Dead zone for analog sticks
        self.gamepad_axis_active = {
            'axis_0': False, # Left stick X (Servo 0)
            'axis_1': False, # Right stick Y (Servo 1)
            # Add other axes/buttons if they control servos
        }


        self.init_ui()
        self.init_ros_publisher()
        self.select_first_available_gamepad()
        '''
        LISTENING WHEN APP IS IN THE BACKGROUND
        # Uncomment if you want global keyboard listening (requires 'keyboard' module, potentially admin rights)
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
            "SERWO #3", # Assuming this is 180 based on previous code structure
            "POMPKA #2" # User requested 360 behavior
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

            # Add reset button only for 360° servos (index 0 and index 3)
            if i == 0: #or i == 3 or i == 2:
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
                if i == 0:
                    reset_btn.clicked.connect(self.reset_servo_0) # Connect to reset_servo_0
                #elif i == 3:
                #    reset_btn.clicked.connect(self.reset_servo_3) # Connect to reset_servo_3
                #elif i == 2:
                #    reset_btn.clicked.connect(self.reset_servo_2) # Connect to reset_servo_3
                control_layout.addWidget(reset_btn)

            control_layout.addWidget(increase_btn)
            column_layout.addLayout(control_layout)

            # Add frame to layout
            servo_layout.addWidget(frame)

            # Connect signals
            self.update_buttons[i].clicked.connect(lambda _, i=i: self.update_step_value(i))

            if i == 0:
                # Special handling for 360° Camera (momentary control, reset on release)
                self.increase_buttons[i].pressed.connect(lambda: self.adjust_servo_position(0, self.step_values[0]))
                self.increase_buttons[i].released.connect(self.reset_servo_0)
                self.decrease_buttons[i].pressed.connect(lambda: self.adjust_servo_position(0, -self.step_values[0]))
                self.decrease_buttons[i].released.connect(self.reset_servo_0)

            else:
                # Standard handling for 180° servos (step on click)
                self.increase_buttons[i].clicked.connect(lambda _, i=i: self.adjust_servo_position(i, self.step_values[i]))
                self.decrease_buttons[i].clicked.connect(lambda _, i=i: self.adjust_servo_position(i, -self.step_values[i]))

            '''
            elif i == 3:
                 # Special handling for 360° Servo 2 (momentary control, reset on release)
                self.increase_buttons[i].pressed.connect(lambda: self.adjust_servo_position(3, self.step_values[3]))
                self.increase_buttons[i].released.connect(self.reset_servo_3)
                self.decrease_buttons[i].pressed.connect(lambda: self.adjust_servo_position(3, -self.step_values[3]))
                self.decrease_buttons[i].released.connect(self.reset_servo_3)
            elif i == 2:
                 # Special handling for 360° Servo 2 (momentary control, reset on release)
                self.increase_buttons[i].pressed.connect(lambda: self.adjust_servo_position(2, self.step_values[2]))
                self.increase_buttons[i].released.connect(self.reset_servo_2)
                self.decrease_buttons[i].pressed.connect(lambda: self.adjust_servo_position(2, -self.step_values[2]))
                self.decrease_buttons[i].released.connect(self.reset_servo_2)
            '''

            

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
        # Updated keyboard instructions to reflect momentary control if applicable
        instruction_label = QLabel('''
            <div style='font-size: 13px; color: #AAA;'>
            <b>KEYBOARD CONTROL:</b><br>
            <table width='100%'>
            <tr><td>360° Camera:</td><td>A (left, held) / D (right, held)</td></tr>
            <tr><td>180° Camera:</td><td>W (up, click) / S (down, click)</td></tr>
            <tr><td>JEDNO Z SERW (180°):</td><td>K (up, click) / L (down, click)</td></tr>
            <tr><td>DRUGIE Z SERW (360°):</td><td>U (up, held) / J (down, held)</td></tr>
            </table>
            *Held keys reset servo to default on release.*
            </div>
        ''')
        instruction_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        instruction_layout.addWidget(instruction_label)

        # Default positions settings frame
        default_pos_frame = QFrame()
        default_pos_frame.setFrameShape(QFrame.Shape.StyledPanel)
        default_pos_frame.setStyleSheet("padding: 6px;")

        default_pos_layout = QVBoxLayout(default_pos_frame)
        default_pos_layout.setSpacing(10)

        default_pos_label = QLabel("<b>DEFAULT POSITIONS</b>")
        default_pos_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        default_pos_layout.addWidget(default_pos_label)

        # Input for 360 Camera default position
        servo0_input_layout = QHBoxLayout()
        servo0_input_layout.addWidget(QLabel("360° Camera:"))
        self.servo0_default_input = QLineEdit(str(self.default_servo_0_pos))
        self.servo0_default_input.setMaximumWidth(60)
        self.servo0_default_input.setAlignment(Qt.AlignmentFlag.AlignCenter)
        servo0_input_layout.addWidget(self.servo0_default_input)
        default_pos_layout.addLayout(servo0_input_layout)
        '''
        servo2_input_layout = QHBoxLayout()
        servo2_input_layout.addWidget(QLabel("360° Servo 1:"))
        self.servo2_default_input = QLineEdit(str(self.default_servo_2_pos))
        self.servo2_default_input.setMaximumWidth(60)
        self.servo2_default_input.setAlignment(Qt.AlignmentFlag.AlignCenter)
        servo2_input_layout.addWidget(self.servo2_default_input)
        default_pos_layout.addLayout(servo2_input_layout)

         # Input for 360 Servo 2 default position
        servo3_input_layout = QHBoxLayout()
        servo3_input_layout.addWidget(QLabel("360° Servo 2:"))
        self.servo3_default_input = QLineEdit(str(self.default_servo_3_pos))
        self.servo3_default_input.setMaximumWidth(60)
        self.servo3_default_input.setAlignment(Qt.AlignmentFlag.AlignCenter)
        servo3_input_layout.addWidget(self.servo3_default_input)
        default_pos_layout.addLayout(servo3_input_layout)
        '''

        # Update button for default positions
        update_defaults_btn = QPushButton("Update Defaults")
        update_defaults_btn.setStyleSheet("""
            QPushButton {
                background-color: #505050;
                border: 1px solid #666;
            }
            QPushButton:hover {
                background-color: #606060;
            }
        """)
        update_defaults_btn.clicked.connect(self.update_default_positions)
        default_pos_layout.addWidget(update_defaults_btn)


        # Add frames to bottom row
        bottom_row.addWidget(instruction_frame)
        bottom_row.addWidget(default_pos_frame)

        gamepad_layout.addWidget(self.gamepad_toggle_button)
        gamepad_layout.addLayout(bottom_row)
        gamepad_group.setLayout(gamepad_layout)

        main_layout.addWidget(gamepad_group)
        main_layout.addItem(QSpacerItem(20, 20, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))

        self.setLayout(main_layout)

    def update_default_positions(self):
        """Update the default positions for 360 servos"""
        try:
            new_servo0_default = int(self.servo0_default_input.text())
            if self.min_360_angle <= new_servo0_default <= self.max_360_angle:
                self.default_servo_0_pos = new_servo0_default
                # If servo 0 is currently at a temporary position, reset it immediately
                if self.servo_positions[0] != self.default_servo_0_pos:
                     self.reset_servo_0()
            else:
                print(f"360° Camera default position should be between {self.min_360_angle} and {self.max_360_angle}")
        except ValueError:
            print("Invalid 360° Camera default position value. Please enter a valid integer.")
        '''
        try:
            new_servo3_default = int(self.servo3_default_input.text())
            if self.min_360_angle <= new_servo3_default <= self.max_360_angle:
                self.default_servo_3_pos = new_servo3_default
                 # If servo 3 is currently at a temporary position, reset it immediately
                if self.servo_positions[3] != self.default_servo_3_pos:
                     self.reset_servo_3()
            else:
                 print(f"360° Servo 2 default position should be between {self.min_360_angle} and {self.max_360_angle}")
        except ValueError:
            print("Invalid 360° Servo 2 default position value. Please enter a valid integer.")


        try:
            new_servo2_default = int(self.servo2_default_input.text())
            if self.min_360_angle <= new_servo2_default <= self.max_360_angle:
                self.default_servo_2_pos = new_servo2_default
                 # If servo 3 is currently at a temporary position, reset it immediately
                if self.servo_positions[2] != self.default_servo_2_pos:
                     self.reset_servo_2()
            else:
                 print(f"360° Servo 1 default position should be between {self.min_360_angle} and {self.max_360_angle}")
        except ValueError:
            print("Invalid 360° Servo 1 default position value. Please enter a valid integer.")
        '''
    def reset_servo_0(self):
        """Reset 360° Camera (index 0) to its default position"""
        self.servo_positions[0] = self.default_servo_0_pos
        self.servo_labels[0].setText(f"Position: {self.servo_positions[0]}°")
        self.publish_servo_positions()

    '''
    def reset_servo_2(self):
        """Reset 360° Camera (index 0) to its default position"""
        self.servo_positions[2] = self.default_servo_2_pos
        self.servo_labels[2].setText(f"Position: {self.servo_positions[2]}°")
        self.publish_servo_positions()

    def reset_servo_3(self):
        """Reset 4th servo (index 3) to its default position"""
        self.servo_positions[3] = self.default_servo_3_pos
        self.servo_labels[3].setText(f"Position: {self.servo_positions[3]}°")
        self.publish_servo_positions()
    '''

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
            # Allow larger steps for 360 servos, but cap 180 servos
            if index == 0: #or index == 3 or index == 2: # 360 servos
                 if new_value >= 1: # Allow any positive step for 360
                    self.step_values[index] = new_value
                 else:
                    print("Step value must be at least 1.")
            else: # 180 servos
                if 1 <= new_value <= 90: # Cap step for 180 servos to 90
                    self.step_values[index] = new_value
                else:
                    print("Step value for 180° servos should be between 1 and 90")

        except ValueError:
            print("Invalid step value. Please enter a valid integer.")
    
    def adjust_servo_position(self, index, delta):
        """Adjusts the servo position based on index and delta"""
        new_position = self.servo_positions[index] + delta

        if index == 0: #or index == 3 or index == 2: # 360 degree servos
            # Clamp the commanded angle within the defined range
            if new_position < self.min_360_angle:
                 new_position = self.min_360_angle
            elif new_position > self.max_360_angle:
                 new_position = self.max_360_angle
            # Note: A true continuous 360 servo might interpret positions
            # differently or expect speed commands. This implementation
            # sends a target angle within the defined +/-360 range.
        else: # 180 degree servos
            # Clamp the angle between 0 and 180
            if new_position < self.min_180_angle:
                new_position = self.min_180_angle
            elif new_position > self.max_180_angle:
                new_position = self.max_180_angle

        self.servo_positions[index] = new_position

        self.servo_labels[index].setText(f"Position: {self.servo_positions[index]}°")
        self.publish_servo_positions()
    
    def publish_servo_positions(self):
        msg = Int32MultiArray()
        # Ensure we are always sending 4 integer positions
        msg.data = [int(p) for p in self.servo_positions[:4]]
        # print(f"Publishing: {msg.data}") # Optional: for debugging
        self.servo_publisher.publish(msg)
    
    def read_gamepad(self):
        """Reads gamepad input and controls servos if active."""
        # pygame.init() # Should be initialized before calling this
        # if self.selected_gamepad:
        #    pygame.joystick.init()
        # else:
        #    return # Exit if no gamepad

        clock = pygame.time.Clock()

        while self.running and self.selected_gamepad:
            pygame.event.pump() # Process pygame events

            if not self.is_gamepad_active:
                 time.sleep(0.1) # Sleep if not active
                 continue # Skip control logic

            current_time = time.time()

            # Check axes for continuous-like control (Servos 0 and 1 in this example)
            # Note: This uses the *current* axis value. For step-by-step like the buttons,
            # you'd check if the axis *crossed* the deadzone threshold since the last check.
            # The current implementation adjusts position continuously while axis is held beyond deadzone.
            if current_time - self.last_gamepad_update >= self.gamepad_update_interval:
                axis_0 = self.selected_gamepad.get_axis(3)  # Left stick X (360° servo 0)
                axis_1 = self.selected_gamepad.get_axis(4)  # Right stick Y (180° camera 1)
                # Add axis for servo 3 if controlling with axis
                # axis_3 = self.selected_gamepad.get_axis(X) # Replace X with correct axis number

                # Handle 360° servo (index 0) using axis 0
                if abs(axis_0) > self.gamepad_axis_dead_zone:
                     # Adjust position continuously while axis is held
                     self.adjust_servo_position(0, int(axis_0 * self.step_values[0]))
                     self.gamepad_axis_active['axis_0'] = True
                else:
                     if self.gamepad_axis_active['axis_0']:
                         # Reset only when the axis returns to the deadzone
                         self.reset_servo_0()
                         self.gamepad_axis_active['axis_0'] = False

                # Handle 180° servo (index 1) using axis 1
                # Note: For 180 servos controlled by axis, continuous adjustment might be desired,
                # or step-on-threshold. This code does continuous adjustment.
                if abs(axis_1) > self.gamepad_axis_dead_zone:
                     self.adjust_servo_position(1, -int(axis_1 * self.step_values[1] * 0.1)) # Reduced step impact for smoother axis control
                     self.gamepad_axis_active['axis_1'] = True
                else:
                     self.gamepad_axis_active['axis_1'] = False # No reset needed for 180 click-style servos

                # Handle 360° servo (index 3) - add logic if controlled by an axis
                # Example for axis 2 (assuming axis 2 controls servo 3)
                # axis_3 = self.selected_gamepad.get_axis(2) # Assume axis 2 for servo 3
                # if abs(axis_3) > self.gamepad_axis_dead_zone:
                #     self.adjust_servo_position(3, int(axis_3 * self.step_values[3]))
                #     self.gamepad_axis_active['axis_3'] = True
                # else:
                #     if self.gamepad_axis_active['axis_3']:
                #         self.reset_servo_3()
                #         self.gamepad_axis_active['axis_3'] = False


                self.last_gamepad_update = current_time

            # Check buttons for step/momentary control
            # Example: Button 0 for Servo 2 Up, Button 1 for Servo 2 Down (180 servo)
            # button_0 = self.selected_gamepad.get_button(0)
            # button_1 = self.selected_gamepad.get_button(1)
            # if button_0:
            #     self.adjust_servo_position(2, self.step_values[2])
            # if button_1:
            #      self.adjust_servo_position(2, -self.step_values[2])

            # Example: Button 2 for Servo 3 Up, Button 3 for Servo 3 Down (360 servo - momentary)
            # button_2 = self.selected_gamepad.get_button(2)
            # button_3 = self.selected_gamepad.get_button(3)
            # if button_2 and not self.gamepad_buttons_pressed.get('button_2', False):
            #      self.adjust_servo_position(3, self.step_values[3])
            #      self.gamepad_buttons_pressed['button_2'] = True
            # elif not button_2 and self.gamepad_buttons_pressed.get('button_2', False):
            #      self.reset_servo_3()
            #      self.gamepad_buttons_pressed['button_2'] = False

            # if button_3 and not self.gamepad_buttons_pressed.get('button_3', False):
            #      self.adjust_servo_position(3, -self.step_values[3])
            #      self.gamepad_buttons_pressed['button_3'] = True
            # elif not button_3 and self.gamepad_buttons_pressed.get('button_3', False):
            #      self.reset_servo_3()
            #      self.gamepad_buttons_pressed['button_3'] = False


            clock.tick(30)  # Limit to 30 FPS
        # pygame.joystick.quit() # Should quit joystick when thread ends


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
            # Optional: Immediately reset 360 servos to default when enabling gamepad control
            # self.reset_servo_0()
            # self.reset_servo_3()
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
             # Optional: Immediately reset 360 servos to default when disabling gamepad control
            self.reset_servo_0()
            self.reset_servo_3()
            self.reset_servo_2()


    def keyPressEvent(self, event):
        # Key press events for 180 servos will adjust position step-by-step (due to key repeat)
        # Key press events for 360 servos will initiate movement that stops on key release
        if event.key() == Qt.Key.Key_W:
            self.adjust_servo_position(1, self.step_values[1])  # 180° Camera - up
        elif event.key() == Qt.Key.Key_S:
            self.adjust_servo_position(1, -self.step_values[1])  # 180° Camera - down
        elif event.key() == Qt.Key.Key_K:
            self.adjust_servo_position(2, self.step_values[2])  # Servo 2 (180) - up
        elif event.key() == Qt.Key.Key_L:
            self.adjust_servo_position(2, -self.step_values[2])  # Servo 2 (180) - down
        # Momentary control initiated on press for 360 servos
        elif event.key() == Qt.Key.Key_A:
            # Only move if not already moving left or right (optional, prevents repeated calls)
            # This is handled by the keyReleaseEvent resetting.
             self.adjust_servo_position(0, -self.step_values[0])  # 360° Camera - left
        elif event.key() == Qt.Key.Key_D:
             self.adjust_servo_position(0, self.step_values[0])  # 360° Camera - right
        elif event.key() == Qt.Key.Key_U:
             self.adjust_servo_position(3, self.step_values[3])  # Servo 3 (360) - up (increase angle value)
        elif event.key() == Qt.Key.Key_J:
             self.adjust_servo_position(3, -self.step_values[3])  # Servo 3 (360) - down (decrease angle value)


    def keyReleaseEvent(self, event):
        # Reset 360 servos when their control keys are released
        if event.key() == Qt.Key.Key_A or event.key() == Qt.Key.Key_D:
            self.reset_servo_0()
        elif event.key() == Qt.Key.Key_U or event.key() == Qt.Key.Key_J:
            self.reset_servo_3()
        elif event.key() == Qt.Key.Key_I or event.key() == Qt.Key.Key_K:
            self.reset_servo_2()

    # Uncomment and use this if global keyboard listening is enabled
    # def listen_for_global_keys(self):
    #     """Listens for global keyboard events (requires 'keyboard' module)."""
    #     # Note: Global key listening might require running the application with administrator privileges
    #     # and can interfere with other keyboard inputs outside the application window.
    #     print("Global keyboard listener started. Press Ctrl+C in console to stop.")
    #     try:
    #         while True:
    #             # Example: check for 'w' press globally
    #             if keyboard.is_pressed('w'):
    #                  # You would likely need a mechanism to only trigger once per press,
    #                  # and potentially handle hold duration or release for 360 servos.
    #                  # This is more complex than the basic keyPressEvent handling.
    #                  # For simple step-on-press, just calling the function might work,
    #                  # but OS key repeat will make it step repeatedly while held.
    #                  # For momentary 360 control like buttons/local keys,
    #                  # you'd need to track key state (pressed/released) globally.
    #                  pass # Add global key handling logic here
    #             time.sleep(0.05) # Check frequently
    #     except Exception as e:
    #         print(f"Global keyboard listener stopped: {e}")


    def closeEvent(self, event):
        self.running = False
        if hasattr(self, 'gamepad_thread') and self.gamepad_thread.is_alive():
            self.gamepad_thread.join()
        # if hasattr(self, 'key_listener_thread') and self.key_listener_thread.is_alive():
        #     # Global listener threads might need specific stopping mechanisms
        #     pass # Implement graceful shutdown if needed
        event.accept()

# Example usage (assuming you have a ROS 2 node and Pygame initialized elsewhere)
# if __name__ == '__main__':
#     import rclpy
#     from PyQt6.QtWidgets import QApplication, QMainWindow
#     import sys

#     rclpy.init(args=None)
#     node = rclpy.create_node('servo_gui_node')

#     pygame.init()
#     pygame.joystick.init()
#     gamepads = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
#     for gamepad in gamepads:
#         gamepad.init()
#         print(f"Initialized gamepad: {gamepad.get_name()}")


#     app = QApplication(sys.argv)
#     main_window = QMainWindow()
#     main_window.setWindowTitle("Servo Control GUI")

#     servo_tab = ServaTab(node=node, gamepads=gamepads)
#     main_window.setCentralWidget(servo_tab)
#     main_window.resize(800, 400)
#     main_window.show()

#     # Use a ROS 2 executor in a separate thread or the main thread
#     # thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
#     # thread.start()

#     try:
#         sys.exit(app.exec())
#     finally:
#         # Shut down ROS 2 node and Pygame
#         # node.destroy_node()
#         # rclpy.shutdown()
#         pygame.quit()