from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox, QGridLayout, QFrame
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Int8MultiArray

class Giz2Tab(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        
        # Initialize states
        self.servo_positions = [150, 50, 90, 0]
        self.output_states = {
            "drill": 0,
            "koszelnik": 0,
            "heater": 0
        }
        self.led_states = [0, 0, 0]  # [manual, autonomy, kill_switch]

        self.init_ros_publishers()
        self.init_ui()

    def init_ros_publishers(self):
        # Publishers for the second ESP32 GIZ
        self.servo_publisher = self.node.create_publisher(
            Int32MultiArray, 
            '/ESP32_GIZ_v2/servo_angles_topic', 
            10
        )
        self.output_publisher = self.node.create_publisher(
            Int8MultiArray, 
            '/ESP32_GIZ_v2/output_state_topic', 
            10
        )
        self.led_publisher = self.node.create_publisher(
            Int8MultiArray, 
            '/ESP32_GIZ_v2/led_state_topic', 
            10
        )

    def init_ui(self):
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(15)


        # Servo Control Section
        servo_group = QGroupBox("Servo Control")
        servo_group.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        servo_layout = QGridLayout()
        
        self.servo_buttons = []
        
        
        for i in range(4):
            frame = QFrame()
            frame.setFrameShape(QFrame.Shape.StyledPanel)
            frame.setLineWidth(1)
            frame_layout = QVBoxLayout(frame)
            
            
            pos0_btn = QPushButton("0°")
            pos0_btn.clicked.connect(lambda _, idx=i: self.set_servo_position(idx, 0))
            
            pos50_btn = QPushButton("50°")
            pos50_btn.clicked.connect(lambda _, idx=i: self.set_servo_position(idx, 50))
            
            pos60_btn = QPushButton("60°")
            pos60_btn.clicked.connect(lambda _, idx=i: self.set_servo_position(idx, 60))

            pos90_btn = QPushButton("90°")
            pos90_btn.clicked.connect(lambda _, idx=i: self.set_servo_position(idx, 90))
            
            pos120_btn = QPushButton("120°")
            pos120_btn.clicked.connect(lambda _, idx=i: self.set_servo_position(idx, 120))

            pos150_btn = QPushButton("150°")
            pos150_btn.clicked.connect(lambda _, idx=i: self.set_servo_position(idx, 150))
            
            pos180_btn = QPushButton("180°")
            pos180_btn.clicked.connect(lambda _, idx=i: self.set_servo_position(idx, 180))

            
            btn_layout = QHBoxLayout()
     
            if i == 0:
                label = QLabel("1: Woda (niebieski)")
                label.setAlignment(Qt.AlignmentFlag.AlignCenter)
                label.setFont(QFont('Arial', 11, QFont.Weight.Bold))
                btn_layout.addWidget(pos150_btn)
                btn_layout.addWidget(pos0_btn)

            if i == 1:
                label = QLabel("2: Apteczka (czerwony)")
                label.setAlignment(Qt.AlignmentFlag.AlignCenter)
                label.setFont(QFont('Arial', 11, QFont.Weight.Bold))
                btn_layout.addWidget(pos50_btn)
                btn_layout.addWidget(pos180_btn)

            if i == 2:
                label = QLabel("3: Amuncija (szary)")
                label.setAlignment(Qt.AlignmentFlag.AlignCenter)
                label.setFont(QFont('Arial', 11, QFont.Weight.Bold))
                btn_layout.addWidget(pos90_btn)
                btn_layout.addWidget(pos0_btn)

            if i == 3:
                label = QLabel("4: Jedzenie (bialy)")
                label.setAlignment(Qt.AlignmentFlag.AlignCenter)
                label.setFont(QFont('Arial', 11, QFont.Weight.Bold))
                btn_layout.addWidget(pos0_btn)
                btn_layout.addWidget(pos180_btn)


            

            frame_layout.addWidget(label)
            frame_layout.addLayout(btn_layout)
            
            servo_layout.addWidget(frame, i//2, i%2)
            self.servo_buttons.append([pos0_btn, pos90_btn, pos180_btn])
        
        servo_group.setLayout(servo_layout)
        main_layout.addWidget(servo_group)

        # Output Control Section
        output_group = QGroupBox("Output Control")
        output_group.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        output_layout = QGridLayout()
        
        # Drill control
        drill_frame = QFrame()
        drill_frame.setFrameShape(QFrame.Shape.StyledPanel)
        drill_frame_layout = QVBoxLayout(drill_frame)
        
        drill_label = QLabel("Drill")
        drill_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        drill_label.setFont(QFont('Arial', 11, QFont.Weight.Bold))
        
        self.drill_on_button = QPushButton("ON")
        self.drill_on_button.clicked.connect(lambda: self.set_output("drill", 1))
        
        self.drill_off_button = QPushButton("OFF")
        self.drill_off_button.clicked.connect(lambda: self.set_output("drill", 0))
        self.drill_off_button.setStyleSheet("background-color: #FF5733;")
        
        drill_btn_layout = QHBoxLayout()
        drill_btn_layout.addWidget(self.drill_on_button)
        drill_btn_layout.addWidget(self.drill_off_button)
        
        drill_frame_layout.addWidget(drill_label)
        drill_frame_layout.addLayout(drill_btn_layout)
        
        output_layout.addWidget(drill_frame, 0, 0)
        
        # Koszelnik control
        koszelnik_frame = QFrame()
        koszelnik_frame.setFrameShape(QFrame.Shape.StyledPanel)
        koszelnik_frame_layout = QVBoxLayout(koszelnik_frame)
        
        koszelnik_label = QLabel("Koszelnik")
        koszelnik_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        koszelnik_label.setFont(QFont('Arial', 11, QFont.Weight.Bold))
        
        self.koszelnik_on_button = QPushButton("ON")
        self.koszelnik_on_button.clicked.connect(lambda: self.set_output("koszelnik", 1))
        
        self.koszelnik_off_button = QPushButton("OFF")
        self.koszelnik_off_button.clicked.connect(lambda: self.set_output("koszelnik", 0))
        self.koszelnik_off_button.setStyleSheet("background-color: #FF5733;")
        
        koszelnik_btn_layout = QHBoxLayout()
        koszelnik_btn_layout.addWidget(self.koszelnik_on_button)
        koszelnik_btn_layout.addWidget(self.koszelnik_off_button)
        
        koszelnik_frame_layout.addWidget(koszelnik_label)
        koszelnik_frame_layout.addLayout(koszelnik_btn_layout)
        
        output_layout.addWidget(koszelnik_frame, 0, 1)
        
        # Heater control
        heater_frame = QFrame()
        heater_frame.setFrameShape(QFrame.Shape.StyledPanel)
        heater_frame_layout = QVBoxLayout(heater_frame)
        
        heater_label = QLabel("Heater")
        heater_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        heater_label.setFont(QFont('Arial', 11, QFont.Weight.Bold))
        
        self.heater_on_button = QPushButton("ON")
        self.heater_on_button.clicked.connect(lambda: self.set_output("heater", 1))
        
        self.heater_off_button = QPushButton("OFF")
        self.heater_off_button.clicked.connect(lambda: self.set_output("heater", 0))
        self.heater_off_button.setStyleSheet("background-color: #FF5733;")
        
        heater_btn_layout = QHBoxLayout()
        heater_btn_layout.addWidget(self.heater_on_button)
        heater_btn_layout.addWidget(self.heater_off_button)
        
        heater_frame_layout.addWidget(heater_label)
        heater_frame_layout.addLayout(heater_btn_layout)
        
        output_layout.addWidget(heater_frame, 1, 0)
        
        output_group.setLayout(output_layout)
        main_layout.addWidget(output_group)

        # LED Control (Control States)
        led_group = QGroupBox("Control States")
        led_group.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        led_layout = QGridLayout()
        
        # Manual control
        manual_frame = QFrame()
        manual_frame.setFrameShape(QFrame.Shape.StyledPanel)
        manual_frame_layout = QVBoxLayout(manual_frame)
        
        manual_label = QLabel("Manual Control")
        manual_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        manual_label.setFont(QFont('Arial', 11, QFont.Weight.Bold))
        
        self.manual_on_button = QPushButton("Enable")
        self.manual_on_button.clicked.connect(lambda: self.set_led_state(0, 1))
        
        self.manual_off_button = QPushButton("Disable")
        self.manual_off_button.clicked.connect(lambda: self.set_led_state(0, 0))
        self.manual_off_button.setStyleSheet("background-color: #FF5733;")
        
        manual_btn_layout = QHBoxLayout()
        manual_btn_layout.addWidget(self.manual_on_button)
        manual_btn_layout.addWidget(self.manual_off_button)
        
        manual_frame_layout.addWidget(manual_label)
        manual_frame_layout.addLayout(manual_btn_layout)
        
        led_layout.addWidget(manual_frame, 0, 0)
        
        # Autonomy control
        autonomy_frame = QFrame()
        autonomy_frame.setFrameShape(QFrame.Shape.StyledPanel)
        autonomy_frame_layout = QVBoxLayout(autonomy_frame)
        
        autonomy_label = QLabel("Autonomy")
        autonomy_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        autonomy_label.setFont(QFont('Arial', 11, QFont.Weight.Bold))
        
        self.autonomy_on_button = QPushButton("Enable")
        self.autonomy_on_button.clicked.connect(lambda: self.set_led_state(1, 1))
        
        self.autonomy_off_button = QPushButton("Disable")
        self.autonomy_off_button.clicked.connect(lambda: self.set_led_state(1, 0))
        self.autonomy_off_button.setStyleSheet("background-color: #FF5733;")
        
        autonomy_btn_layout = QHBoxLayout()
        autonomy_btn_layout.addWidget(self.autonomy_on_button)
        autonomy_btn_layout.addWidget(self.autonomy_off_button)
        
        autonomy_frame_layout.addWidget(autonomy_label)
        autonomy_frame_layout.addLayout(autonomy_btn_layout)
        
        led_layout.addWidget(autonomy_frame, 0, 1)
        
        # Kill switch control
        kill_switch_frame = QFrame()
        kill_switch_frame.setFrameShape(QFrame.Shape.StyledPanel)
        kill_switch_frame_layout = QVBoxLayout(kill_switch_frame)
        
        kill_switch_label = QLabel("Kill Switch")
        kill_switch_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        kill_switch_label.setFont(QFont('Arial', 11, QFont.Weight.Bold))
        
        self.kill_switch_on_button = QPushButton("Activate")
        self.kill_switch_on_button.clicked.connect(lambda: self.set_led_state(2, 1))
        self.kill_switch_on_button.setStyleSheet("background-color: red;")
        
        self.kill_switch_off_button = QPushButton("Deactivate")
        self.kill_switch_off_button.clicked.connect(lambda: self.set_led_state(2, 0))
        
        kill_switch_btn_layout = QHBoxLayout()
        kill_switch_btn_layout.addWidget(self.kill_switch_on_button)
        kill_switch_btn_layout.addWidget(self.kill_switch_off_button)
        
        kill_switch_frame_layout.addWidget(kill_switch_label)
        kill_switch_frame_layout.addLayout(kill_switch_btn_layout)
        
        led_layout.addWidget(kill_switch_frame, 1, 0)
        
        led_group.setLayout(led_layout)
        main_layout.addWidget(led_group)
        
        # Publish all states button
        self.publish_all_button = QPushButton("Publish All States")
        self.publish_all_button.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        self.publish_all_button.setMinimumHeight(50)
        self.publish_all_button.clicked.connect(self.publish_all_states)
        self.publish_all_button.setStyleSheet("background-color: #2ECC71; color: white;")
        main_layout.addWidget(self.publish_all_button)

        self.setLayout(main_layout)
        self.apply_styles()

    def set_servo_position(self, index, position):
        self.servo_positions[index] = position
        self.publish_servo_positions()
        
        # Update button styling
        for i, buttons in enumerate(self.servo_buttons):
            if i == index:
                pos_idx = 7  # 0, 1, or 2 for 0°, 90°, 180°
                for j, btn in enumerate(buttons):
                    if j == pos_idx:
                        btn.setStyleSheet("background-color: #2ECC71;")
                    else:
                        btn.setStyleSheet("")

    def set_output(self, output_name, state):
        self.output_states[output_name] = state
        self.publish_output_states()
        
        # Update button styling
        if output_name == "drill":
            self.drill_on_button.setStyleSheet("background-color: #2ECC71;" if state else "")
            self.drill_off_button.setStyleSheet("" if state else "background-color: #FF5733;")
        elif output_name == "koszelnik":
            self.koszelnik_on_button.setStyleSheet("background-color: #2ECC71;" if state else "")
            self.koszelnik_off_button.setStyleSheet("" if state else "background-color: #FF5733;")
        elif output_name == "heater":
            self.heater_on_button.setStyleSheet("background-color: #2ECC71;" if state else "")
            self.heater_off_button.setStyleSheet("" if state else "background-color: #FF5733;")

    def set_led_state(self, index, state):
        self.led_states[index] = state
        self.publish_led_states()
        
        # Update button styling
        if index == 0:  # Manual
            self.manual_on_button.setStyleSheet("background-color: #2ECC71;" if state else "")
            self.manual_off_button.setStyleSheet("" if state else "background-color: #FF5733;")
        elif index == 1:  # Autonomy
            self.autonomy_on_button.setStyleSheet("background-color: #2ECC71;" if state else "")
            self.autonomy_off_button.setStyleSheet("" if state else "background-color: #FF5733;")
        elif index == 2:  # Kill Switch
            self.kill_switch_on_button.setStyleSheet("background-color: red;" if state else "")
            self.kill_switch_off_button.setStyleSheet("background-color: #2ECC71;" if not state else "")

    def publish_servo_positions(self):
        msg = Int32MultiArray()
        msg.data = self.servo_positions
        self.servo_publisher.publish(msg)
        print(f"Published servo positions: {self.servo_positions}")

    def publish_output_states(self):
        msg = Int8MultiArray()
        msg.data = [
            self.output_states["drill"],
            self.output_states["koszelnik"],
            self.output_states["heater"]
        ]
        self.output_publisher.publish(msg)
        print(f"Published output states: {msg.data}")

    def publish_led_states(self):
        msg = Int8MultiArray()
        msg.data = self.led_states
        self.led_publisher.publish(msg)
        print(f"Published LED states: {self.led_states}")

    def publish_all_states(self):
        self.publish_servo_positions()
        self.publish_output_states()
        self.publish_led_states()
        print("Published all states")

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
                min-width: 80px;
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