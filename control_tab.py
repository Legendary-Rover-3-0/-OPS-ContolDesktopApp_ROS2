from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                            QPushButton, QComboBox, QSlider)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPixmap

class ControlTab(QWidget):
    def __init__(self, gamepads, toggle_manual_callback, toggle_kill_switch_callback, 
                 toggle_autonomy_callback, update_speed_factor_callback):
        super().__init__()
        self.gamepads = gamepads
        self.toggle_kill_switch_callback = toggle_kill_switch_callback
        self.toggle_autonomy_callback = toggle_autonomy_callback
        self.toggle_manual_callback = toggle_manual_callback
        self.update_speed_factor_callback = update_speed_factor_callback
        
        self.init_ui()

    def init_ui(self):
        main_layout = QHBoxLayout()

        # Lewa kolumna: Model łazika
        left_column = QVBoxLayout()
        rover_label = QLabel()
        rover_pixmap = QPixmap('newRover.png')
        rover_label.setPixmap(rover_pixmap.scaled(250, 250, Qt.AspectRatioMode.KeepAspectRatio))
        left_column.addWidget(rover_label)
        left_column.addWidget(QLabel('Model łazika'))
        left_column.setSpacing(10)
        left_column.setContentsMargins(10, 10, 10, 10)
        main_layout.addLayout(left_column, stretch=3)  

        # Środkowa kolumna: Model manipulatora
        middle_column = QVBoxLayout()
        manipulator_label = QLabel()
        manipulator_pixmap = QPixmap('newArm.png')
        manipulator_label.setPixmap(manipulator_pixmap.scaled(250, 250, Qt.AspectRatioMode.KeepAspectRatio))
        middle_column.addWidget(manipulator_label)
        middle_column.addWidget(QLabel('Model manipulatora'))
        middle_column.setSpacing(10)
        middle_column.setContentsMargins(10, 10, 10, 10)
        main_layout.addLayout(middle_column, stretch=3)

        # Prawa kolumna: Kontrolki i logo
        right_column = QVBoxLayout()

        # Sekcja wyboru pada
        gamepad_section = QVBoxLayout()
        self.label = QLabel('Wybierz pada i naciśnij przycisk, aby wysłać dane przez ROS2')
        gamepad_section.addWidget(self.label)

        self.gamepad_selector = QComboBox()
        for i, gamepad in enumerate(self.gamepads):
            self.gamepad_selector.addItem(gamepad.get_name(), i)
        gamepad_section.addWidget(self.gamepad_selector)

        # Dodajemy suwak do regulacji prędkości
        speed_section = QVBoxLayout()
        speed_section.addWidget(QLabel('Maksymalna prędkość:'))
        
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(10, 100)  # 10% do 100%
        self.speed_slider.setValue(100)  # Domyślnie 100%
        self.speed_slider.setTickInterval(10)
        self.speed_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        
        self.speed_label = QLabel('100%')
        self.speed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        speed_section.addWidget(self.speed_slider)
        speed_section.addWidget(self.speed_label)
        gamepad_section.addLayout(speed_section)

        right_column.addLayout(gamepad_section)

        # Sekcja przycisków sterowania
        control_buttons = QVBoxLayout()
        control_buttons.addWidget(QLabel('Sterowanie:'))

        self.manual_drive_button = QPushButton('Manual Drive: OFF')
        self.manual_drive_button.clicked.connect(self.toggle_manual_callback)
        self.style_button(self.manual_drive_button, '#FF5733')
        control_buttons.addWidget(self.manual_drive_button)

        self.autonomy_button = QPushButton('Autonomy Drive: OFF')
        self.autonomy_button.clicked.connect(self.toggle_autonomy_callback)
        self.style_button(self.autonomy_button, '#FF5733')
        control_buttons.addWidget(self.autonomy_button)

        self.kill_switch_button = QPushButton('Kill Switch: OFF')
        self.kill_switch_button.clicked.connect(self.toggle_kill_switch_callback)
        self.style_button(self.kill_switch_button, '#FF5733')
        control_buttons.addWidget(self.kill_switch_button)

        right_column.addLayout(control_buttons)

        # Sekcja logo
        logo_section = QVBoxLayout()
        logo_label = QLabel()
        logo_pixmap = QPixmap('logo.png')
        logo_label.setPixmap(logo_pixmap.scaled(300, 300, Qt.AspectRatioMode.KeepAspectRatio))
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
