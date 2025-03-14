import sys
import pygame
import threading
from PyQt6.QtWidgets import QApplication, QMainWindow, QTabWidget
from PyQt6.QtGui import QPixmap, QImage, QPalette, QColor
from PyQt6.QtCore import QTimer, Qt
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from control_tab import ControlTab
from vision_tab import VisionTab, CameraWindow
from science_tab import ScienceTab
from status_tab import StatusTab
from serva_tab import ServaTab
from gps_tab import GPSTab
from mani_tab import ManipulatorTab
from ros_node import ROSNode
import rclpy
import config

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
        self.control_tab = ControlTab(self.gamepads, self.toggle_manual_callback, self.toggle_kill_switch, self.toggle_autonomy)
        self.vision_tab = VisionTab(self.open_camera_window)
        self.ros_node = ROSNode(self.update_image)
        self.science_tab = ScienceTab(self.ros_node)
        self.status_tab = StatusTab()
        self.serva_tab = ServaTab(self.ros_node, self.gamepads)
        self.gps_tab = GPSTab(self.ros_node)
        self.mani_tab = ManipulatorTab(self.ros_node, self.gamepads)

        self.tabs.addTab(self.control_tab, 'Sterowanie')
        self.tabs.addTab(self.vision_tab, 'Wizja')
        self.tabs.addTab(self.science_tab, 'Science')
        self.tabs.addTab(self.status_tab, 'Status Jetsona')
        self.tabs.addTab(self.gps_tab, 'GPS')
        self.tabs.addTab(self.serva_tab, 'Serva')
        self.tabs.addTab(self.mani_tab, 'Manipulator')
        self.setCentralWidget(self.tabs)

        # Ustawienie QoS dla subskrypcji obrazów
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        for topic in config.CAMERA_TOPICS:
            self.ros_node.add_camera_subscription(topic, qos_profile)

        # Uruchomienie timera do aktualizacji obrazów
        self.timer = QTimer()
        self.timer.timeout.connect(lambda: rclpy.spin_once(self.ros_node, timeout_sec=0.01))
        self.timer.start(30)

        self.kill_switch_state = 0
        self.autonomy_state = 0
        self.manual_drive_state = 0
        self.camera_windows = [None] * 4

    def update_image(self, cv_image, idx):
        # Przekazanie obrazu do VisionTab
        self.vision_tab.update_image(cv_image, idx)

    def toggle_kill_switch(self):
        self.kill_switch_state = 1 - self.kill_switch_state
        self.autonomy_state = 0
        self.manual_drive_state = 0
        self.control_tab.update_button_state(self.control_tab.manual_drive_button, 'Manual Drive', self.manual_drive_state)
        self.control_tab.update_button_state(self.control_tab.autonomy_button, 'Autonomy Drive', self.autonomy_state)
        self.control_tab.update_button_state(self.control_tab.kill_switch_button, 'Kill Switch', self.kill_switch_state)
        self.ros_node.publish_button_states(self.kill_switch_state, self.autonomy_state, self.manual_drive_state)

    def toggle_autonomy(self):
        self.autonomy_state = 1 - self.autonomy_state
        self.kill_switch_state = 0
        self.manual_drive_state = 0
        self.control_tab.update_button_state(self.control_tab.manual_drive_button, 'Manual Drive', self.manual_drive_state)
        self.control_tab.update_button_state(self.control_tab.autonomy_button, 'Autonomy Drive', self.autonomy_state)
        self.control_tab.update_button_state(self.control_tab.kill_switch_button, 'Kill Switch', self.kill_switch_state)
        self.ros_node.publish_button_states(self.kill_switch_state, self.autonomy_state, self.manual_drive_state)

    def toggle_manual_callback(self):
        self.manual_drive_state = 1 - self.manual_drive_state
        self.kill_switch_state = 0
        self.autonomy_state = 0
        self.control_tab.update_button_state(self.control_tab.manual_drive_button, 'Manual Drive', self.manual_drive_state)
        self.control_tab.update_button_state(self.control_tab.autonomy_button, 'Autonomy Drive', self.autonomy_state)
        self.control_tab.update_button_state(self.control_tab.kill_switch_button, 'Kill Switch', self.kill_switch_state)
        self.ros_node.publish_button_states(self.kill_switch_state, self.autonomy_state, self.manual_drive_state)
        if self.manual_drive_state == 1:
            self.start_reading()
            
    def start_reading(self):
        index = self.control_tab.gamepad_selector.currentData()
        if index is not None:
            self.selected_gamepad = pygame.joystick.Joystick(index)
            self.selected_gamepad.init()
            self.control_tab.label.setText(f'Wybrano: {self.selected_gamepad.get_name()}')
            self.mani_tab.set_selected_gamepad(self.selected_gamepad)  # Przekazanie wybranego pada do ManipulatorTab
            if self.reading_thread is None or not self.reading_thread.is_alive():
                self.running = True
                self.reading_thread = threading.Thread(target=self.read_gamepad, daemon=True)
                self.reading_thread.start()

    def read_gamepad(self):
        while self.running:
            if self.manual_drive_state == 0:
                for _ in range(5):
                    self.ros_node.publish_empty_gamepad_input()
                    pygame.time.wait(50)
                self.running = False
                return
            
            else:
                pygame.event.pump()
                buttons = [self.selected_gamepad.get_button(i) for i in range(self.selected_gamepad.get_numbuttons())]
                axes = [self.selected_gamepad.get_axis(i) for i in range(min(6, self.selected_gamepad.get_numaxes()))]
                hat = self.selected_gamepad.get_hat(0)  # Pobranie wartości D-pad (hat)

                self.ros_node.publish_gamepad_input(buttons, axes, hat)  # Teraz hat jest dodawany do axes
            
            pygame.time.wait(50)

    def open_camera_window(self, idx):
        if self.camera_windows[idx] is None:
            self.camera_windows[idx] = CameraWindow(self.vision_tab.camera_labels[idx], f'Kamera {idx + 1}')
        self.camera_windows[idx].show()
        self.camera_windows[idx].activateWindow()