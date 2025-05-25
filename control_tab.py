from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                            QPushButton, QComboBox, QSlider, QGroupBox,)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPixmap
from PyQt6.QtGui import QFont
import config
import serial.tools.list_ports
from PyQt6.QtCore import QObject, QThread, pyqtSignal

class PortScannerWorker(QObject):
    ports_found = pyqtSignal(list)

    def run(self):
        ports = serial.tools.list_ports.comports()
        usb_ports = [port.device for port in ports if port.device.startswith("/dev/ttyUSB")]
        self.ports_found.emit(usb_ports)

class ControlTab(QWidget):
    def __init__(self, gamepads, connect_satel, toggle_communication_callback,
                 toggle_manual_callback, toggle_kill_switch_callback, 
                 toggle_autonomy_callback, update_speed_factor_callback):
        super().__init__()
        self.gamepads = gamepads
        self.toggle_kill_switch_callback = toggle_kill_switch_callback
        self.toggle_autonomy_callback = toggle_autonomy_callback
        self.toggle_manual_callback = toggle_manual_callback
        self.update_speed_factor_callback = update_speed_factor_callback
        self.connect_satel = connect_satel
        self.toggle_communication_callback = toggle_communication_callback
        
        self.init_ui()

    def init_ui(self):
        main_layout = QHBoxLayout()

        # Lewa kolumna: modelele + logo pod spodem
        left_column = QVBoxLayout()

        # Modele w jednej linii
        models_row = QHBoxLayout()

        # Model łazika
        rover_layout = QVBoxLayout()
        rover_label = QLabel()
        rover_pixmap = QPixmap('newRover.png')
        rover_label.setPixmap(rover_pixmap.scaled(250, 250, Qt.AspectRatioMode.KeepAspectRatio))
        rover_layout.addWidget(rover_label)
        rover_layout.addWidget(QLabel('Model łazika'))
        models_row.addLayout(rover_layout)

        # Model manipulatora
        manipulator_layout = QVBoxLayout()
        manipulator_label = QLabel()
        manipulator_pixmap = QPixmap('newArm.png')
        manipulator_label.setPixmap(manipulator_pixmap.scaled(250, 250, Qt.AspectRatioMode.KeepAspectRatio))
        manipulator_layout.addWidget(manipulator_label)
        manipulator_layout.addWidget(QLabel('Model manipulatora'))
        models_row.addLayout(manipulator_layout)

        # Dodaj wiersz modeli do kolumny
        left_column.addLayout(models_row)

        # Logo na dole lewej kolumny
        logo_label = QLabel()
        logo_pixmap = QPixmap('logo.png')
        logo_label.setPixmap(logo_pixmap.scaled(300, 300, Qt.AspectRatioMode.KeepAspectRatio))
        left_column.addWidget(logo_label, alignment=Qt.AlignmentFlag.AlignCenter)

        left_column.setSpacing(10)
        left_column.setContentsMargins(10, 10, 10, 10)

        main_layout.addLayout(left_column, stretch=3)

        # Prawa kolumna: Kontrolki
        right_column = QVBoxLayout()
        gamepad_section = QVBoxLayout()

        # ------------------------------
        # Grupa: Port szeregowy
        # ------------------------------
        serial_group = QGroupBox("Port szeregowy")
        serial_group.setFont(QFont('Arial', 11, QFont.Weight.Bold))
        serial_layout = QVBoxLayout()

        # Wiersz: port i przycisk odświeżania
        port_row = QHBoxLayout()
        port_row.addWidget(QLabel("Port szeregowy:"))
        self.serial_port_selector = QComboBox()
        port_row.addWidget(self.serial_port_selector)
        self.refresh_button = QPushButton("🔄️")
        self.refresh_button.setToolTip("Odśwież porty")
        self.refresh_button.setFixedWidth(30)
        self.refresh_button.clicked.connect(self.refresh_serial_ports)
        port_row.addWidget(self.refresh_button)
        serial_layout.addLayout(port_row)

        # Wiersz: Baudrate
        baudrate_row = QHBoxLayout()
        baudrate_row.addWidget(QLabel("Baudrate:"))
        self.baudrate_input = QComboBox()
        self.baudrate_input.addItems(["9600", "115200", "57600", "38400", "19200", "4800"])
        self.baudrate_input.setEditable(True)
        self.baudrate_input.setCurrentText("9600")
        baudrate_row.addWidget(self.baudrate_input)
        serial_layout.addLayout(baudrate_row)

        # Przycisk połącz
        self.connect_serial_button = QPushButton("Connect")
        self.connect_serial_button.clicked.connect(self.connect_satel)
        serial_layout.addWidget(self.connect_serial_button)

        serial_group.setLayout(serial_layout)
        right_column.addWidget(serial_group)

        # ------------------------------
        # Grupa: Pad i prędkość
        # ------------------------------
        pad_group = QGroupBox("Pad i prędkość")
        pad_group.setFont(QFont('Arial', 11, QFont.Weight.Bold))
        pad_layout = QVBoxLayout()

        # Wybór pada
        self.label = QLabel('Wybierz pada i naciśnij przycisk, aby wysłać dane przez ROS2')
        pad_layout.addWidget(self.label)

        self.gamepad_selector = QComboBox()
        for i, gamepad in enumerate(self.gamepads):
            self.gamepad_selector.addItem(gamepad.get_name(), i)
        pad_layout.addWidget(self.gamepad_selector)

        # Suwak prędkości
        speed_section = QVBoxLayout()
        speed_section.addWidget(QLabel('Maksymalna prędkość:'))

        speed_row = QHBoxLayout()
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(10, 100)
        self.speed_slider.setValue(100)
        self.speed_slider.setTickInterval(10)
        self.speed_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        speed_row.addWidget(self.speed_slider)

        self.speed_label = QLabel('100%')
        self.speed_label.setFixedWidth(40)
        self.speed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        speed_row.addWidget(self.speed_label)

        speed_section.addLayout(speed_row)
        pad_layout.addLayout(speed_section)

        pad_group.setLayout(pad_layout)
        right_column.addWidget(pad_group)

        # ------------------------------
        # Grupa: Sterowanie
        # ------------------------------
        control_group = QGroupBox("Sterowanie")
        control_group.setFont(QFont('Arial', 11, QFont.Weight.Bold))
        control_layout = QVBoxLayout()

        # Przycisk ROS2
        self.communication_button = QPushButton('Communication: ROS2')
        self.communication_button.clicked.connect(self.toggle_communication_callback)
        self.style_button(self.communication_button, config.BUTTON_DEFAULT_COLOR)
        control_layout.addWidget(self.communication_button)

        # Wiersz: Manual + Autonomy
        mode_buttons_row = QHBoxLayout()

        self.manual_drive_button = QPushButton('Manual Drive: OFF')
        self.manual_drive_button.clicked.connect(self.toggle_manual_callback)
        self.style_button(self.manual_drive_button, '#FF5733')
        mode_buttons_row.addWidget(self.manual_drive_button)

        self.autonomy_button = QPushButton('Autonomy Drive: OFF')
        self.autonomy_button.clicked.connect(self.toggle_autonomy_callback)
        self.style_button(self.autonomy_button, '#FF5733')
        mode_buttons_row.addWidget(self.autonomy_button)

        control_layout.addLayout(mode_buttons_row)

        # Przycisk Kill Switch
        self.kill_switch_button = QPushButton('Kill Switch: OFF')
        self.kill_switch_button.clicked.connect(self.toggle_kill_switch_callback)
        self.style_button(self.kill_switch_button, '#FF5733')
        control_layout.addWidget(self.kill_switch_button)

        control_group.setLayout(control_layout)
        right_column.addWidget(control_group)

        # Opcjonalnie: rozpychacz na dole, jeśli chcesz spiąć wszystko do góry
        right_column.addStretch()

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

    def refresh_serial_ports(self):
        self.serial_port_selector.clear()
        self.serial_port_selector.addItem("Skanowanie...")

        self.port_thread = QThread()
        self.port_worker = PortScannerWorker()
        self.port_worker.moveToThread(self.port_thread)

        self.port_thread.started.connect(self.port_worker.run)
        self.port_worker.ports_found.connect(self.update_serial_ports)
        self.port_worker.ports_found.connect(self.port_thread.quit)
        self.port_worker.ports_found.connect(self.port_worker.deleteLater)
        self.port_thread.finished.connect(self.port_thread.deleteLater)

        self.port_thread.start()

    def update_serial_ports(self, port_list):
        self.serial_port_selector.clear()
        if port_list:
            self.serial_port_selector.addItems(port_list)
        else:
            self.serial_port_selector.addItem("Brak portów")


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
            QGroupBox {{
                font-weight: bold;
                border: 2px solid #555;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 15px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }}
        """)

    def adjust_color(self, color, factor):
        from PyQt6.QtGui import QColor
        qcolor = QColor(color)
        return qcolor.lighter(int(100 * factor)).name()
