import sys
import pygame
import threading
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox, QTabWidget, QMainWindow, QFrame, QScrollArea
)
from PyQt6.QtGui import QPixmap, QImage, QPalette, QColor
from PyQt6.QtCore import QTimer, Qt
from ros_node import ROSNode
import rclpy
import config

# Sekcja: CameraWindow
class CameraWindow(QWidget):
    def __init__(self, camera_label, title):
        super().__init__()
        self.setWindowTitle(title)
        self.setGeometry(200, 200, 640, 480)
        self.setMinimumSize(200, 150)  # Ustaw minimalny rozmiar okna
        layout = QVBoxLayout()
        self.label = QLabel()
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)  # Wyśrodkuj obraz
        self.label.setScaledContents(False)  # Wyłącz automatyczne skalowanie
        self.label.setStyleSheet("background-color: black;")  # Czarny kolor tła
        layout.addWidget(self.label)
        self.setLayout(layout)
        self.camera_label = camera_label
        self.update_image()

    def update_image(self):
        pixmap = self.camera_label.pixmap()
        if pixmap:
            # Skaluj obraz z zachowaniem proporcji
            scaled_pixmap = pixmap.scaled(self.label.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
            self.label.setPixmap(scaled_pixmap)
        QTimer.singleShot(30, self.update_image)

    def resizeEvent(self, event):
        self.update_image()
        super().resizeEvent(event)

# Sekcja: MainWindow
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        pygame.init()

        self.setWindowTitle('Stacja Operatorska 2.2')
        self.setGeometry(100, 100, 800, 600)
        self.setStyleSheet("background-color: #1e1e1e; color: white;")

        self.gamepads = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
        self.selected_gamepad = None
        self.reading_thread = None
        self.running = False

        self.tabs = QTabWidget()
        self.control_tab = QWidget()
        self.vision_tab = QWidget()
        self.science_tab = QWidget()
        self.status_tab = QWidget()

        self.init_control_tab()
        self.init_vision_tab()
        self.init_science_tab()
        self.init_status_tab()

        self.tabs.addTab(self.control_tab, 'Sterowanie')
        self.tabs.addTab(self.vision_tab, 'Wizja')
        self.tabs.addTab(self.science_tab, 'Science')
        self.tabs.addTab(self.status_tab, 'Status Jetsona')
        self.setCentralWidget(self.tabs)

        self.ros_node = ROSNode(self.update_image)
        for topic in config.CAMERA_TOPICS:  # Użyj tematów z config.py
            self.ros_node.add_camera_subscription(topic)

        self.timer = QTimer()
        self.timer.timeout.connect(lambda: rclpy.spin_once(self.ros_node, timeout_sec=0.01))
        self.timer.start(30)

        self.kill_switch_state = 0
        self.autonomy_state = 0
        self.camera_windows = [None] * 4

    # Sekcja: Control Tab
    def init_control_tab(self):
        main_layout = QHBoxLayout()

        # Lewa kolumna: Miejsce na modele robotów 
        left_column = QVBoxLayout()
        left_column.addWidget(QLabel('Model Robota 1'))  # Tymczasowy tekst
        left_column.addWidget(QWidget())  # Tymczasowe puste miejsce
        left_column.setSpacing(10)
        left_column.setContentsMargins(10, 10, 10, 10)
        main_layout.addLayout(left_column, stretch=3)  

        # Środkowa kolumna: Miejsce na modele robotów 
        middle_column = QVBoxLayout()
        middle_column.addWidget(QLabel('Model Robota 2'))  # Tymczasowy tekst
        middle_column.addWidget(QWidget())  # Tymczasowe puste miejsce
        middle_column.setSpacing(10)
        middle_column.setContentsMargins(10, 10, 10, 10)
        main_layout.addLayout(middle_column, stretch=3)

        # Prawa kolumna: Kontrolki i logo
        right_column = QVBoxLayout()

        # Sekcja wyboru pada
        gamepad_section = QVBoxLayout()
        self.label = QLabel('Wybierz pada i naciśnij przycisk, aby wysłać dane przez ROS2')  # Dodane self.label
        gamepad_section.addWidget(self.label)  # Dodane do layoutu

        self.gamepad_selector = QComboBox()
        for i, gamepad in enumerate(self.gamepads):
            self.gamepad_selector.addItem(gamepad.get_name(), i)
        gamepad_section.addWidget(self.gamepad_selector)

        self.start_button = QPushButton('Rozpocznij publikowanie')
        self.start_button.clicked.connect(self.start_reading)
        self.style_button(self.start_button, '#4CAF50')  # Zielony przycisk
        gamepad_section.addWidget(self.start_button)

        right_column.addLayout(gamepad_section)

        # Sekcja przycisków sterowania
        control_buttons = QVBoxLayout()
        control_buttons.addWidget(QLabel('Sterowanie:'))

        self.kill_switch_button = QPushButton('Kill Switch: OFF')
        self.kill_switch_button.clicked.connect(self.toggle_kill_switch)
        self.style_button(self.kill_switch_button, '#FF5733')
        control_buttons.addWidget(self.kill_switch_button)

        self.autonomy_button = QPushButton('Autonomy: OFF')
        self.autonomy_button.clicked.connect(self.toggle_autonomy)
        self.style_button(self.autonomy_button, '#FF5733')
        control_buttons.addWidget(self.autonomy_button)

        self.extra_button = QPushButton('Extra Button')
        self.style_button(self.extra_button, '#FF5733')
        control_buttons.addWidget(self.extra_button)

        right_column.addLayout(control_buttons)

        # Sekcja logo
        logo_section = QVBoxLayout()
        logo_label = QLabel()
        logo_pixmap = QPixmap('logo.png')
        logo_label.setPixmap(logo_pixmap.scaled(150, 150, Qt.AspectRatioMode.KeepAspectRatio))
        logo_section.addWidget(logo_label, alignment=Qt.AlignmentFlag.AlignCenter)

        right_column.addLayout(logo_section)

        right_column.setSpacing(20)
        right_column.setContentsMargins(10, 10, 10, 10)
        main_layout.addLayout(right_column, stretch=4)  # 40% szerokości

        self.control_tab.setLayout(main_layout)

    # Sekcja: Vision Tab
    def init_vision_tab(self):
        layout = QVBoxLayout()
        self.vision_tabs = QTabWidget()
        self.camera_labels = []
        for i in range(4):
            tab = QWidget()
            tab_layout = QVBoxLayout()
            label = QLabel(f'Oczekiwanie na obraz z kamery {i + 1}...')
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)  # Wyśrodkuj tekst
            label.setStyleSheet("background-color: black; color: white;")  # Czarny kolor tła
            label.setScaledContents(False)  # Wyłącz automatyczne skalowanie
            self.camera_labels.append(label)
            open_button = QPushButton(f'Otwórz kamerę {i + 1}')
            open_button.clicked.connect(lambda _, idx=i: self.open_camera_window(idx))
            self.style_button(open_button, '#3498DB')  # Niebieski przycisk
            tab_layout.addWidget(label)
            tab_layout.addWidget(open_button)
            tab.setLayout(tab_layout)
            self.vision_tabs.addTab(tab, f'Kamera {i + 1}')
        layout.addWidget(self.vision_tabs)
        self.vision_tab.setLayout(layout)

    # Sekcja: Science Tab
    def init_science_tab(self):
        layout = QVBoxLayout()
        self.science_tab.setLayout(layout)

    # Sekcja: Status Tab
    def init_status_tab(self):
        layout = QVBoxLayout()
        self.status_tab.setLayout(layout)

    # Sekcja: Update Image
    def update_image(self, cv_image, idx):
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_img = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format.Format_RGB888).rgbSwapped()
        pixmap = QPixmap.fromImage(q_img)
        
        # Skaluj obraz z zachowaniem proporcji
        scaled_pixmap = pixmap.scaled(self.camera_labels[idx].size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
        self.camera_labels[idx].setPixmap(scaled_pixmap)
        
        if self.camera_windows[idx]:
            scaled_pixmap_window = pixmap.scaled(self.camera_windows[idx].label.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
            self.camera_windows[idx].label.setPixmap(scaled_pixmap_window)

    # Sekcja: Open Camera Window
    def open_camera_window(self, idx):
        if self.camera_windows[idx] is None:
            self.camera_windows[idx] = CameraWindow(self.camera_labels[idx], f'Kamera {idx + 1}')
        self.camera_windows[idx].show()
        self.camera_windows[idx].activateWindow()

    # Sekcja: Toggle Kill Switch
    def toggle_kill_switch(self):
        self.kill_switch_state = 1 - self.kill_switch_state
        self.update_button_state(self.kill_switch_button, 'Kill Switch', self.kill_switch_state)
        self.ros_node.publish_button_states(self.kill_switch_state, self.autonomy_state)

    # Sekcja: Toggle Autonomy
    def toggle_autonomy(self):
        self.autonomy_state = 1 - self.autonomy_state
        self.update_button_state(self.autonomy_button, 'Autonomy', self.autonomy_state)
        self.ros_node.publish_button_states(self.kill_switch_state, self.autonomy_state)

    # Sekcja: Start Reading
    def start_reading(self):
        index = self.gamepad_selector.currentData()
        if index is not None:
            self.selected_gamepad = pygame.joystick.Joystick(index)
            self.selected_gamepad.init()
            self.label.setText(f'Wybrano: {self.selected_gamepad.get_name()}')
            if self.reading_thread is None or not self.reading_thread.is_alive():
                self.running = True
                self.reading_thread = threading.Thread(target=self.read_gamepad, daemon=True)
                self.reading_thread.start()
                self.style_button(self.start_button, '#2ECC71')  # Zielony przycisk po uruchomieniu

    # Sekcja: Read Gamepad
    def read_gamepad(self):
        while self.running:
            pygame.event.pump()
            buttons = [self.selected_gamepad.get_button(i) for i in range(self.selected_gamepad.get_numbuttons())]
            axes = [self.selected_gamepad.get_axis(i) for i in range(min(6, self.selected_gamepad.get_numaxes()))]
            self.ros_node.publish_gamepad_input(buttons, axes)
            pygame.time.wait(50)

    # Sekcja: Style Button
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

    # Sekcja: Adjust Color
    def adjust_color(self, color, factor):
        # Funkcja do przyciemniania/rozjaśniania kolorów
        from PyQt6.QtGui import QColor
        qcolor = QColor(color)
        return qcolor.lighter(int(100 * factor)).name()

    # Sekcja: Update Button State
    def update_button_state(self, button, text, state):
        color = '#2ECC71' if state else '#FF5733'  # Zielony/Czerwony
        button.setText(f'{text}: {"ON" if state else "OFF"}')
        self.style_button(button, color)

# Sekcja: Main
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())