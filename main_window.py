import sys
import time
import pygame
import threading
from PyQt6.QtWidgets import QApplication, QMainWindow, QTabWidget
from PyQt6.QtGui import QPixmap, QImage, QPalette, QColor
from PyQt6.QtCore import QTimer, Qt
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from control_tab import ControlTab
from science_tab import ScienceTab
from status_tab import StatusTab
from serva_tab import ServaTab
from gps_tab import GPSTab
from mani_tab import ManipulatorTab
from gvision_tab import CamerasTab
from ros_node import ROSNode
from keyboard_tab import KeyboardTab
from giz2_tab import GIZ2Tab
#from radiation_map_tab import RadiationMapTab
import rclpy
import config
import serial

class MainWindow(QMainWindow):
    def closeEvent(self, event):
        print("Zamykanie aplikacji...")
        if self.manual_drive_state == 1 or self.mani_tab.is_gamepad_active == 1:
            self.manual_drive_state = 0  # Wymuszenie zatrzymania ręcznego trybu jazdy
            self.mani_tab.is_gamepad_active = 0
            self.serva_tab.is_gamepad_active = 0
            print("Zatrzymywanie lazika, zeby nie zabil kogos lub siebie...")
            time.sleep(1)

        if config.AUTO_CLOSE_SERVOS_ON_APP_CLOSE:
            self.science_tab.close_all_servos()
            
        event.accept()  # Zamknij aplikację normalnie

    def __init__(self):
        super().__init__()
        pygame.init()

        self.setWindowTitle('Stacja Operatorska 2.2')
        self.setGeometry(100, 100, 1024, 768)  # Zwiększony rozmiar okna
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e1e;
            }
            QTabWidget::pane {
                border: 1px solid #444;
                background: #2d2d2d;
            }
            QTabBar::tab {
                background: #333;
                color: white;
                padding: 12px 20px;
                border: 1px solid #444;
                border-bottom: none;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                font-size: 14px;
                min-width: 120px;
            }
            QTabBar::tab:selected {
                background: #505050;
                border-color: #666;
            }
            QTabBar::tab:hover {
                background: #404040;
            }
        """)

        self.gamepads = []
        self.selected_gamepad = None
        self.reading_thread = None
        self.running = False
        self.gamepad_connected = False

        self.speed_factor = 1.0  # Domyślna wartość 100%

        self.tabs = QTabWidget()
        self.tabs.setTabPosition(QTabWidget.TabPosition.North)
        self.tabs.setMovable(True)
        self.tabs.setDocumentMode(False)
        
        # Zwiększenie czcionki w zakładkach
        font = self.tabs.font()
        font.setPointSize(12)
        self.tabs.setFont(font)

        self.control_tab = ControlTab(self.gamepads, self.connect_satel, self.toggle_communication_callback, self.toggle_manual_callback, self.toggle_kill_switch, self.toggle_autonomy, self.update_speed_factor, self.refresh_gamepads, self.disconnect_gamepad)
        self.ros_node = ROSNode()
        self.serva_tab = ServaTab(self.ros_node, self.gamepads)
        self.science_tab = ScienceTab(self.ros_node, self.serva_tab)
        self.status_tab = StatusTab()
        self.gps_tab = GPSTab(self.ros_node)
        self.mani_tab = ManipulatorTab(self.ros_node, self.gamepads)
        self.gvision_tab = CamerasTab()
        self.kayboard_tab = KeyboardTab(self.ros_node)
        self.giz2_tab = None  # Placeholder for future use
        #self.radiation_map_tab = RadiationMapTab()

        self.tabs.addTab(self.control_tab, 'Sterowanie')
        self.tabs.addTab(self.mani_tab, 'Manipulator')
        self.tabs.addTab(self.gvision_tab, 'Wizja')
        self.tabs.addTab(self.science_tab, 'Science')
        self.tabs.addTab(self.status_tab, 'Status Jetsona')
        self.tabs.addTab(self.gps_tab, 'GPS')
        self.tabs.addTab(self.serva_tab, 'Serva')
        self.tabs.addTab(self.kayboard_tab, 'Klawiatura')
        #self.tabs.addTab(self.radiation_map_tab, 'Radiation Map')

        self.setCentralWidget(self.tabs)

        # Uruchomienie timera do aktualizacji obrazów i sprawdzania stanu pada
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(30)

        self.kill_switch_state = 0
        self.autonomy_state = 0
        self.manual_drive_state = 0
        self.camera_windows = [None] * 4

        # Inicjalizacja listy gamepadów
        self.refresh_gamepads()

    def update_status(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0.01)
        self.check_gamepad_connection()

    def refresh_gamepads(self):
        """Odświeża listę podłączonych gamepadów"""
        pygame.joystick.quit()
        pygame.joystick.init()
        self.gamepads = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
        self.control_tab.update_gamepad_list(self.gamepads)

    def check_gamepad_connection(self):
        """Sprawdza, czy wybrany gamepad jest nadal podłączony"""
        if self.selected_gamepad is not None and not self.gamepad_connected:
            # Jeśli gamepad był podłączony, ale teraz nie ma
            self.handle_gamepad_disconnected()

    def handle_gamepad_disconnected(self):
        """Obsługa sytuacji gdy gamepad zostanie odłączony"""
        if self.manual_drive_state == 1:
            print("Gamepad odłączony - kill switch ON")
            self.toggle_kill_switch()  # Wyłączenie manual drive
            
        self.selected_gamepad = None
        self.gamepad_connected = False
        self.control_tab.label.setText("Brak podłączonego pada")
        self.control_tab.disconnect_button.setEnabled(False)
        self.refresh_gamepads()

    def disconnect_gamepad(self):
        """Rozłącza aktualnie podłączony gamepad"""
        if self.selected_gamepad is not None:
            if self.manual_drive_state == 1:
                self.toggle_kill_switch()  # Wyłączenie manual drive
                
            self.selected_gamepad = None
            self.gamepad_connected = False
            self.control_tab.label.setText("Brak podłączonego pada")
            self.control_tab.disconnect_button.setEnabled(False)
            print("Gamepad rozłączony ręcznie")

    def connect_satel(self):
        """Ustaw nowy port szeregowy"""

        if self.ros_node.serial_port is not None:
            # disconnect
            self.ros_node.serial_port = None
            self.control_tab.style_button(self.control_tab.connect_serial_button, config.BUTTON_DEFAULT_COLOR)
            self.control_tab.connect_serial_button.setText("Connect")
            if self.ros_node.communication_mode == 'SATEL':
                self.toggle_communication_callback()

            print(f"Rozczono port.")
            return

        selected_port = self.control_tab.serial_port_selector.currentText()
        try:
            selected_baudrate = int(self.control_tab.baudrate_input.currentText())
        except ValueError:
            selected_baudrate = 9600
            self.control_tab.baudrate_input.setCurrentText("9600")

        try:
            self.ros_node.serial_port = serial.Serial(
                port=selected_port,
                baudrate=selected_baudrate,
                timeout=1
            )
            print(f"Połączono z {selected_port} @ {selected_baudrate} baud.")
        except serial.SerialException as e:
            print(f"Nie udało się otworzyć portu {selected_port}: {e}")
            self.ros_node.serial_port = None

        if self.ros_node.serial_port is not None:
            self.control_tab.style_button(self.control_tab.connect_serial_button, config.BUTTON_ON_COLOR)
            self.control_tab.connect_serial_button.setText("Successfully Connected!")
        else:
            self.control_tab.style_button(self.control_tab.connect_serial_button, config.BUTTON_OFF_COLOR)
            self.control_tab.connect_serial_button.setText("Error: Not Connected!")

    def toggle_communication_callback(self):
        # Wylaczenie skryptu jazdy autonomicznej
        if self.ros_node.communication_mode == "ROS2" and self.ros_node.serial_port is not None:
            self.ros_node.communication_mode = "SATEL"
            self.control_tab.style_button(self.control_tab.communication_button, config.BUTTON_SELECTED_COLOR)

        elif self.ros_node.communication_mode == "SATEL":
            self.ros_node.communication_mode = "ROS2"
            self.control_tab.style_button(self.control_tab.communication_button, config.BUTTON_DEFAULT_COLOR)
            
        self.control_tab.communication_button.setText(f"Communication: {self.ros_node.communication_mode}")

    def toggle_kill_switch(self):
        # Wylaczenie skryptu jazdy autonomicznej
        if self.autonomy_state:
            self.status_tab.stop_autonomy_drive()

        self.kill_switch_state = 1 - self.kill_switch_state
        self.autonomy_state = 0
        self.manual_drive_state = 0
        self.control_tab.update_button_state(self.control_tab.manual_drive_button, 'Manual Drive', self.manual_drive_state)
        self.control_tab.update_button_state(self.control_tab.autonomy_button, 'Autonomy Drive', self.autonomy_state)
        self.control_tab.update_button_state(self.control_tab.kill_switch_button, 'Kill Switch', self.kill_switch_state)
        self.ros_node.publish_button_states(self.kill_switch_state, self.autonomy_state, self.manual_drive_state)

        #Aktualizacja stanu gamepada w ManiTab i ServaTab
        self.mani_tab.is_gamepad_active = self.manual_drive_state == 1
        self.serva_tab.is_gamepad_active = self.manual_drive_state == 1

        for _ in range(5):
            self.ros_node.publish_empty_gamepad_input()
            self.mani_tab.publish_empty_values()
            time.sleep(0.05)

    def toggle_autonomy(self):
        self.autonomy_state = 1 - self.autonomy_state
        self.kill_switch_state = 0
        self.manual_drive_state = 0
        self.control_tab.update_button_state(self.control_tab.manual_drive_button, 'Manual Drive', self.manual_drive_state)
        self.control_tab.update_button_state(self.control_tab.autonomy_button, 'Autonomy Drive', self.autonomy_state)
        self.control_tab.update_button_state(self.control_tab.kill_switch_button, 'Kill Switch', self.kill_switch_state)
        self.ros_node.publish_button_states(self.kill_switch_state, self.autonomy_state, self.manual_drive_state)

        # Aktualizacja stanu gamepada w ManiTab i ServaTab
        self.mani_tab.is_gamepad_active = self.manual_drive_state == 1
        self.serva_tab.is_gamepad_active = self.manual_drive_state == 1

        for _ in range(5):
            self.ros_node.publish_empty_gamepad_input()
            self.mani_tab.publish_empty_values()
            time.sleep(0.05)

        # Uruchomienie skryptu jazdy autonomicznej
        if self.autonomy_state:
            self.status_tab.start_autonomy_drive()
        else:
            self.status_tab.stop_autonomy_drive()

    def toggle_manual_callback(self):
        # Wylaczenie skryptu jazdy autonomicznej
        if self.autonomy_state:
            self.status_tab.stop_autonomy_drive()

        self.manual_drive_state = 1 - self.manual_drive_state
        self.kill_switch_state = 0
        self.autonomy_state = 0
        self.control_tab.update_button_state(self.control_tab.manual_drive_button, 'Manual Drive', self.manual_drive_state)
        self.control_tab.update_button_state(self.control_tab.autonomy_button, 'Autonomy Drive', self.autonomy_state)
        self.control_tab.update_button_state(self.control_tab.kill_switch_button, 'Kill Switch', self.kill_switch_state)
        self.ros_node.publish_button_states(self.kill_switch_state, self.autonomy_state, self.manual_drive_state)
        
        # Aktualizacja stanu gamepada w ManiTab i ServaTab
        self.mani_tab.is_gamepad_active = self.manual_drive_state == 1
        self.serva_tab.is_gamepad_active = self.manual_drive_state == 1
        
        if self.manual_drive_state == 1:
            self.start_reading()
        else:
            self.running = False
            for _ in range(5):
                self.ros_node.publish_empty_gamepad_input()
                self.mani_tab.publish_empty_values()
                time.sleep(0.05)
                
    def start_reading(self):
        index = self.control_tab.gamepad_selector.currentData()
        if index is not None:
            try:
                self.selected_gamepad = pygame.joystick.Joystick(index)
                self.selected_gamepad.init()
                self.gamepad_connected = True
                self.control_tab.label.setText(f'Wybrano: {self.selected_gamepad.get_name()}')
                self.control_tab.disconnect_button.setEnabled(True)
                
                # Przekazanie wybranego gamepada do ManiTab i ServaTab
                self.mani_tab.set_selected_gamepad(self.selected_gamepad)
                self.serva_tab.set_selected_gamepad(self.selected_gamepad)
                
                if self.reading_thread is None or not self.reading_thread.is_alive():
                    self.running = True
                    self.reading_thread = threading.Thread(target=self.read_gamepad)
                    self.reading_thread.start()
            except pygame.error as e:
                print(f"Błąd podczas inicjalizacji gamepada: {e}")
                self.handle_gamepad_disconnected()

    def read_gamepad(self):
        pygame.event.set_allowed([pygame.JOYDEVICEREMOVED])
        
        while self.running and self.manual_drive_state == 1:
            try:
                for event in pygame.event.get():
                    if event.type == pygame.JOYDEVICEREMOVED:
                        print(f"Gamepad odłączony")
                        self.handle_gamepad_disconnected()
                        break
                
                if self.selected_gamepad is None or not self.gamepad_connected:
                    self.ros_node.publish_empty_gamepad_input()
                    self.mani_tab.publish_empty_values()
                    time.sleep(0.1)
                    continue
                    
                # Normalne odczytywanie danych gamepada
                buttons = [self.selected_gamepad.get_button(i) for i in range(self.selected_gamepad.get_numbuttons())]
                axes = [self.selected_gamepad.get_axis(i) for i in range(min(6, self.selected_gamepad.get_numaxes()))]
                hat = self.selected_gamepad.get_hat(0)

                self.ros_node.publish_gamepad_input(buttons, axes, hat)
                pygame.time.wait(50)
                
            except pygame.error as e:
                print(f"Błąd pygame: {e}")
                self.handle_gamepad_disconnected()
                break
            except Exception as e:
                print(f"Nieoczekiwany błąd: {e}")
                break

    # def is_gamepad_connected(self):
    #     """Sprawdza czy wybrany gamepad jest nadal podłączony"""
    #     try:
    #         # Próba odczytu nazwy gamepada - jeśli się nie uda, to gamepad jest odłączony
    #         _ = self.selected_gamepad.get_name()
    #         return True
    #     except pygame.error:
    #         return False

    def update_speed_factor(self, factor):
        """Aktualizuje współczynnik prędkości w ROSNode"""
        self.speed_factor = factor
        self.ros_node.update_speed_factor(factor)
