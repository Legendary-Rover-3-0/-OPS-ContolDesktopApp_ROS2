from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                            QPushButton, QComboBox, QSlider, QLineEdit, QGroupBox, QFormLayout)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPixmap
import time # Import time for potential delays between movements if needed later

class ControlTab(QWidget):
    def __init__(self, gamepads, toggle_manual_callback, toggle_kill_switch_callback,
                 toggle_autonomy_callback, update_speed_factor_callback):
        super().__init__()
        self.gamepads = gamepads
        self.toggle_kill_switch_callback = toggle_kill_switch_callback
        self.toggle_autonomy_callback = toggle_autonomy_callback
        self.toggle_manual_callback = toggle_manual_callback
        self.update_speed_factor_callback = update_speed_factor_callback
        #self.ros_node = ros_node

        self.move_timer = QTimer(self)
        self.move_timer.timeout.connect(self.stop_movement)

        # --- Autonomia v.2 Configurations (Edit this list) ---
        # Each dictionary defines a button and its corresponding movement
        self.autonomia_v2_configs = [
            {
                "name": "Autonomia v.2.1",
                "linear_x": 0.2,
                "angular_z": 0.0,
                "duration": 7.0
            },
            {
                "name": "Autonomia v.2.2",
                "linear_x": 0.2,
                "angular_z": 0.0,
                "duration": 9.0
            },
            {
                "name": "Autonomia v.2.3",
                "linear_x": 0.2,
                "angular_z": 0.0,
                "duration": 10.0
            },
            {
                "name": "Autonomia v.2.4",
                "linear_x": 0.2,
                "angular_z": 0.0,
                "duration": 11.0
            },
            {
                "name": "Autonomia v.2.5",
                "linear_x": 0.2,
                "angular_z": 0.0,
                "duration": 24.0
            },
        ]

        self.init_ui()

    def init_ui(self):
        main_layout = QHBoxLayout()

        # Lewa kolumna: Model łazika
        left_column = QVBoxLayout()
        rover_label = QLabel()
        try:
            rover_pixmap = QPixmap('rover_image.png')
            rover_label.setPixmap(rover_pixmap.scaled(250, 250, Qt.AspectRatioMode.KeepAspectRatio))
        except:
            rover_label.setText("Could not load rover_image.png")
            rover_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left_column.addWidget(rover_label)
        rover_title_label = QLabel('Model łazika')
        rover_title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left_column.addWidget(rover_title_label)
        left_column.setSpacing(10)
        left_column.setContentsMargins(10, 10, 10, 10)
        main_layout.addLayout(left_column, stretch=3)

        # Środkowa kolumna: Model manipulatora
        middle_column = QVBoxLayout()
        manipulator_label = QLabel()
        try:
            manipulator_pixmap = QPixmap('manipulator_new.png')
            manipulator_label.setPixmap(manipulator_pixmap.scaled(250, 250, Qt.AspectRatioMode.KeepAspectRatio))
        except:
            manipulator_label.setText("Could not load manipulator_new.png")
            manipulator_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        middle_column.addWidget(manipulator_label)
        manipulator_title_label = QLabel('Model manipulatora')
        manipulator_title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        middle_column.addWidget(manipulator_title_label)
        middle_column.setSpacing(10)
        middle_column.setContentsMargins(10, 10, 10, 10)
        main_layout.addLayout(middle_column, stretch=3)

        # Prawa kolumna: Kontrolki i logo
        right_column = QVBoxLayout()

        # Sekcja wyboru pada
        gamepad_group = QGroupBox("Ustawienia Gamepada")
        gamepad_layout = QVBoxLayout()
        self.label = QLabel('Wybierz pada i naciśnij przycisk, aby wysłać dane przez ROS2')
        gamepad_layout.addWidget(self.label)

        self.gamepad_selector = QComboBox()
        for i, gamepad in enumerate(self.gamepads):
            self.gamepad_selector.addItem(gamepad.get_name(), i)
        gamepad_layout.addWidget(self.gamepad_selector)
        gamepad_group.setLayout(gamepad_layout)
        right_column.addWidget(gamepad_group)

        # Sekcja regulacji prędkości
        speed_group = QGroupBox("Regulacja Prędkości Maksymalnej")
        speed_layout = QVBoxLayout()
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(10, 100)  # 10% do 100%
        self.speed_slider.setValue(100)  # Domyślnie 100%
        self.speed_slider.setTickInterval(10)
        self.speed_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.speed_slider.valueChanged.connect(self.on_speed_changed)

        self.speed_label = QLabel('100%')
        self.speed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        speed_layout.addWidget(self.speed_slider)
        speed_layout.addWidget(self.speed_label)
        speed_group.setLayout(speed_layout)
        right_column.addWidget(speed_group)


        # Sekcja przycisków sterowania trybem
        mode_control_group = QGroupBox("Kontrola Trybu")
        mode_control_layout = QVBoxLayout()

        self.manual_drive_button = QPushButton('Manual Drive: OFF')
        self.manual_drive_button.clicked.connect(self.toggle_manual_callback)
        self.style_button(self.manual_drive_button, '#FF5733')
        mode_control_layout.addWidget(self.manual_drive_button)

        self.autonomy_button = QPushButton('Autonomy Drive: OFF')
        self.autonomy_button.clicked.connect(self.toggle_autonomy_callback)
        self.style_button(self.autonomy_button, '#FF5733')
        mode_control_layout.addWidget(self.autonomy_button)

        self.kill_switch_button = QPushButton('Kill Switch: OFF')
        self.kill_switch_button.clicked.connect(self.toggle_kill_switch_callback)
        self.style_button(self.kill_switch_button, '#FF5733')
        mode_control_layout.addWidget(self.kill_switch_button)

        mode_control_group.setLayout(mode_control_layout)
        right_column.addWidget(mode_control_group)


        # Sekcja przycisków Autonomia v.2
        autonomia_v2_group = QGroupBox("Autonomia v.2 Scenariusze")
        autonomia_v2_layout = QVBoxLayout()

        # Create buttons based on the configurations
        for config in self.autonomia_v2_configs:
            button = QPushButton(config["name"])
            # Connect each button to the handler, passing its specific config
            button.clicked.connect(lambda checked, cfg=config: self.start_autonomia_v2_scenario(cfg))
            self.style_button(button, '#3498DB') # A different color for these buttons
            autonomia_v2_layout.addWidget(button)

        autonomia_v2_group.setLayout(autonomia_v2_layout)
        right_column.addWidget(autonomia_v2_group)


        # Sekcja logo
        logo_section = QVBoxLayout()
        logo_label = QLabel()
        try:
            logo_pixmap = QPixmap('logo.png')
            logo_label.setPixmap(logo_pixmap.scaled(300, 300, Qt.AspectRatioMode.KeepAspectRatio))
        except:
             logo_label.setText("Could not load logo.png")
             logo_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        logo_section.addWidget(logo_label, alignment=Qt.AlignmentFlag.AlignCenter)

        right_column.addLayout(logo_section)

        right_column.setSpacing(20)
        right_column.setContentsMargins(10, 10, 10, 10)
        main_layout.addLayout(right_column, stretch=4)

        self.setLayout(main_layout)

    def on_speed_changed(self, value):
        """Obsługa zmiany wartości suwaka"""
        self.speed_label.setText(f'{value}%')
        # Przelicz na współczynnik 0.1-1.0 i wywołaj callback
        speed_factor = value / 100.0
        self.update_speed_factor_callback(speed_factor)

    def update_button_state(self, button, text, state):
        color = '#2ECC71' if state else '#FF5733'  # Zielony/Czerwony
        button.setText(f'{text}: {"ON" if state else "OFF"}')
        self.style_button(button, color)

    def style_button(self, button, color):
        button.setStyleSheet(f"""
            QPushButton {{
                background-color: {color};
                color: white;
                font-weight: bold;
                border: none;
                padding: 10px;
                border-radius: 5px;
            }}
            QPushButton:hover {{
                background-color: {self.adjust_color(color, 1.2)};
            }}
            QPushButton:pressed {{
                background-color: {self.adjust_color(color, 0.8)};
            }}
        """)

    def adjust_color(self, color, factor):
        from PyQt6.QtGui import QColor
        qcolor = QColor(color)
        return qcolor.lighter(int(100 * factor)).name()

    def start_autonomia_v2_scenario(self, config):
        linear_x = config.get("linear_x", 0.0) # Get parameters with default values
        angular_z = config.get("angular_z", 0.0)
        duration = config.get("duration", 1.0)
        sequence = config.get("sequence", None) # Check for a sequence of movements

        if sequence:
            # Handle a sequence of movements (more complex, requires a state machine or chaining timers)
            print(f"Starting complex sequence: {config['name']}")
            # This part would need more advanced logic to execute movements sequentially
            # For now, we'll just print a message.
            # Implementing sequences is beyond the scope of this single button request
            # but the data structure is there if you want to build upon it later.
            return
        else:
            # Handle a simple single timed movement
            print(f"Starting simple timed movement: {config['name']} (v_x={linear_x}, v_z={angular_z}, duration={duration}s)")
            if duration <= 0:
                print("Timed movement duration must be greater than 0.")
                return

            self.ros_node.publish_cmd_vel_timed(linear_x, angular_z)

            # Start the timer to stop movement after duration
            self.move_timer.start(int(duration * 1000)) # Timer expects milliseconds


    def stop_movement(self):
        """Stops the timed movement."""
        print("Stopping timed movement.")
        self.ros_node.publish_cmd_vel_timed(0.0, 0.0) # Publish zero velocity
        self.move_timer.stop()