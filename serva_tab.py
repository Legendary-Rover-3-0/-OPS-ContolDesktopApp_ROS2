from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox, QLineEdit
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QPalette
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
        self.is_gamepad_active = False  # Domyślnie wyłączony
        
        self.servo_positions = [84, 90, 90, 90]  # Domyślne pozycje
        self.step_values = [10, 10, 10, 10]  # Domyślne kroki dla serw
        self.first_servo = 84
        
        self.init_ui()
        self.init_ros_publisher()
        self.select_first_available_gamepad()

    def init_ui(self):
        main_layout = QVBoxLayout()
        
        self.servo_group = QGroupBox("Servo Control")
        servo_layout = QHBoxLayout()  # Zmieniamy na QHBoxLayout
        
        self.servo_labels = [QLabel(f'Servo {i+1} Position: {self.servo_positions[i]}°') for i in range(4)]
        self.step_inputs = [QLineEdit(str(self.step_values[i])) for i in range(4)]
        self.update_buttons = [QPushButton(f'Aktualizuj wartość dla serwa {i+1}') for i in range(4)]
        self.increase_buttons = [QPushButton('→') for i in range(4)]
        self.decrease_buttons = [QPushButton('←') for i in range(4)]
        
        for i in range(4):
            column_layout = QVBoxLayout()  # Kolumna dla każdego serwa
            column_layout.addWidget(self.servo_labels[i])
            
            step_layout = QHBoxLayout()
            step_layout.addWidget(self.step_inputs[i])
            step_layout.addWidget(self.update_buttons[i])
            column_layout.addLayout(step_layout)
            
            button_layout = QHBoxLayout()
            button_layout.addWidget(self.decrease_buttons[i])
            button_layout.addWidget(self.increase_buttons[i])
            column_layout.addLayout(button_layout)
            
            servo_layout.addLayout(column_layout)
            
            self.update_buttons[i].clicked.connect(lambda _, i=i: self.update_step_value(i))
            self.increase_buttons[i].clicked.connect(lambda _, i=i: self.adjust_servo_position(i, self.step_values[i]))
            self.decrease_buttons[i].clicked.connect(lambda _, i=i: self.adjust_servo_position(i, -self.step_values[i]))
        
        self.servo_group.setLayout(servo_layout)
        main_layout.addWidget(self.servo_group)

        # Dodajemy przycisk do włączania/wyłączania gamepada
        self.gamepad_toggle_button = QPushButton("Gamepad OFF")
        self.gamepad_toggle_button.setStyleSheet("background-color: red")
        self.gamepad_toggle_button.clicked.connect(self.toggle_gamepad)
        main_layout.addWidget(self.gamepad_toggle_button)

        self.setLayout(main_layout)

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
            self.step_values[index] = int(self.step_inputs[index].text())
        except ValueError:
            print("Invalid step value. Please enter a valid integer.")
    
    def adjust_servo_position(self, index, delta):
        if index == 0:
            # Dla serwa 360, zmieniamy wartość tylko RAZ
            self.servo_positions[index] = self.first_servo + delta  # Ustawiamy nową wartość
            self.servo_labels[index].setText(f'Servo {index+1} Position: {self.servo_positions[index]}°')
            self.publish_servo_positions()
        else:
            # Dla pozostałych serw, działamy normalnie
            self.servo_positions[index] = max(0, min(180, self.servo_positions[index] + delta))
            self.servo_labels[index].setText(f'Servo {index+1} Position: {self.servo_positions[index]}°')
            self.publish_servo_positions()
    
    def publish_servo_positions(self):
        msg = Int32MultiArray()
        msg.data = self.servo_positions
        self.servo_publisher.publish(msg)
    
    def read_gamepad(self):
        while self.running and self.selected_gamepad:
            if self.is_gamepad_active:  # Sprawdzamy, czy gamepad jest aktywny
                pygame.event.pump()
            
                axis_0 = self.selected_gamepad.get_axis(0)  # Oś X (lewy drążek)
                axis_1 = self.selected_gamepad.get_axis(1)  # Oś Y (lewy drążek)
                
                if abs(axis_0) > 0.1:
                    # Dla serwa 360, zmieniamy wartość tylko RAZ
                    if self.servo_positions[0] == self.first_servo:  # Tylko jeśli wartość jest domyślna
                        self.adjust_servo_position(0, int(axis_0 * self.step_values[0]))
                else:
                    # Po zwolnieniu drążka, wracamy do wartości domyślnej (90)
                    if self.servo_positions[0] != self.first_servo:
                        self.servo_positions[0] = self.first_servo
                        self.servo_labels[0].setText(f'Servo 1 Position: {self.servo_positions[0]}°')
                        self.publish_servo_positions()
                
                if abs(axis_1) > 0.1:
                    self.adjust_servo_position(1, int(axis_1 * self.step_values[1]))
            
            pygame.time.wait(100)
    
    def toggle_gamepad(self):
        self.is_gamepad_active = not self.is_gamepad_active
        if self.is_gamepad_active:
            self.gamepad_toggle_button.setText("Gamepad ON")
            self.gamepad_toggle_button.setStyleSheet("background-color: green")
        else:
            self.gamepad_toggle_button.setText("Gamepad OFF")
            self.gamepad_toggle_button.setStyleSheet("background-color: red")

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_W:
            self.adjust_servo_position(1, self.step_values[1])
        elif event.key() == Qt.Key.Key_S:
            self.adjust_servo_position(1, -self.step_values[1])
        elif event.key() == Qt.Key.Key_A:
            self.adjust_servo_position(0, -self.step_values[0])  # Zmniejsz wartość dla serwa 0
        elif event.key() == Qt.Key.Key_D:
            self.adjust_servo_position(0, self.step_values[0])  # Zwiększ wartość dla serwa 0

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key.Key_A or event.key() == Qt.Key.Key_D:
            # Po zwolnieniu przycisków A lub D, wracamy do wartości domyślnej (90)
            self.servo_positions[0] = self.first_servo
            self.servo_labels[0].setText(f'Servo 1 Position: {self.servo_positions[0]}°')
            self.publish_servo_positions()

    def closeEvent(self, event):
        self.running = False
        if hasattr(self, 'gamepad_thread') and self.gamepad_thread.is_alive():
            self.gamepad_thread.join()
        event.accept()