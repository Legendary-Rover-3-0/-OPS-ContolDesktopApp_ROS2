from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QGroupBox, 
    QLineEdit, QLabel, QFrame, QGridLayout, QScrollArea, QTextEdit
)
from PyQt6.QtGui import QFont, QPalette, QColor, QTextCursor, QTextCharFormat, QTextFormat, QKeyEvent
from PyQt6.QtCore import QProcess, Qt, pyqtSignal
from rclpy.node import Node
from std_msgs.msg import String
import re


class KeyboardTab(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.capslock = False
        self.log_history = []
        self.keyboard_publisher = self.node.create_publisher(String, '/keyboard', 10)
        self.init_ui()
        self.apply_styles()
        self.setup_connections()

    def init_ui(self):
        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)

        # Left column - Logs
        left_column = QFrame()
        left_column.setFrameShape(QFrame.Shape.StyledPanel)
        left_column_layout = QVBoxLayout()
        
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setFont(QFont('Arial', 11))
        
        left_column_layout.addWidget(QLabel("History of sent messages"))
        left_column_layout.addWidget(self.log_display)
        left_column.setLayout(left_column_layout)
        left_column.setMinimumWidth(400)

        # Right column - keyboard input
        right_column = QVBoxLayout()
        right_column.setSpacing(15)

        # Send keyboard Group
        send_group = QGroupBox("Send")
        send_group.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        send_layout = QVBoxLayout()
        send_layout.setSpacing(10)

        send_label = QLabel("Enter text:")
        send_label.setFont(QFont('Arial', 11))
        
        input_layout = QHBoxLayout()
        self.text_to_send = QLineEdit()
        self.text_to_send.setFont(QFont('Arial', 11))
        self.text_to_send.setMinimumHeight(35)
        self.text_to_send.textChanged.connect(self.update_prediction)
        self.text_to_send.installEventFilter(self)

        self.send_button = QPushButton("Send")
        self.send_button.setStyleSheet("background-color: #2ECC71;")
        input_layout.addWidget(self.text_to_send)
        input_layout.addWidget(self.send_button)

        # special characters button
        buttons_layout = QGridLayout()
        self.backspace_button = QPushButton("Backspace")
        buttons_layout.addWidget(self.backspace_button, 1, 1)

        self.space_button = QPushButton("Space")
        buttons_layout.addWidget(self.space_button, 1, 2)

        self.capslock_button = QPushButton("Capslock")
        buttons_layout.addWidget(self.capslock_button, 1, 3)

        self.enter_button = QPushButton("Enter")
        buttons_layout.addWidget(self.enter_button, 1, 4)

        send_layout.addWidget(send_label)
        send_layout.addLayout(input_layout)
        send_layout.addLayout(buttons_layout)
        send_group.setLayout(send_layout)
        right_column.addWidget(send_group)
        right_column.addStretch()

        # Prediction group
        predict_group = QGroupBox("Output prediction")
        predict_group.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        predict_layout = QVBoxLayout()
        predict_layout.setSpacing(10)

        self.predict_label = QTextEdit()
        self.predict_label.setReadOnly(True)
        self.predict_label.setFont(QFont('Arial', 11))
        self.predict_label.setMaximumHeight(100)

        self.clear_button = QPushButton("Clear logs and output")
        self.clear_button.setStyleSheet("background-color: #FF5733;")

        predict_layout.addWidget(self.predict_label)
        predict_layout.addWidget(self.clear_button)
        predict_group.setLayout(predict_layout)
        right_column.addWidget(predict_group)
        right_column.addStretch()
        
        main_layout.addWidget(left_column)
        main_layout.addLayout(right_column)
        self.setLayout(main_layout)

    def setup_connections(self):
        self.send_button.clicked.connect(self.send_keyboard_string)
        self.clear_button.clicked.connect(self.clear_logs)
        self.backspace_button.clicked.connect(lambda: self.insert_special_sequence("\\b"))
        self.space_button.clicked.connect(lambda: self.insert_special_sequence("\\s"))
        self.capslock_button.clicked.connect(lambda: self.insert_special_sequence("\\c"))
        self.enter_button.clicked.connect(lambda: self.insert_special_sequence("\\e"))

    def eventFilter(self, source, event):
        if source == self.text_to_send and event.type() == QKeyEvent.Type.KeyPress:
            # Allow only specific keys
            allowed_keys = [
                Qt.Key.Key_Backspace, Qt.Key.Key_Delete, Qt.Key.Key_Left, 
                Qt.Key.Key_Right, Qt.Key.Key_Home, Qt.Key.Key_End
            ]
            
            if event.key() in allowed_keys:
                return False
                
            # Allow only a-z, 0-9 and backslash
            text = event.text()
            if text and not re.match(r'^[a-z0-9\\]$', text):
                return True
                
        return super().eventFilter(source, event)

    def insert_special_sequence(self, sequence):
        current_text = self.text_to_send.text()
        self.text_to_send.setText(current_text + sequence)
        self.text_to_send.setFocus()

    def send_keyboard_string(self):
        text = self.text_to_send.text()
        if not text:
            return
        
        msg = String()
        msg.data = text  # Przypisz tekst do pola data wiadomości String
        self.keyboard_publisher.publish(msg)
        
        self.log_history.append(text)
        self.update_log_display()
        self.text_to_send.clear()
        self.update_prediction_from_history()
        

    def update_log_display(self):
        self.log_display.clear()
        
        # Create default text format
        default_format = QTextCharFormat()
        default_format.setFontWeight(QFont.Weight.Normal)
        default_format.setBackground(QColor('#3a3a3a'))  # Background matching the style
        
        # Create format for special sequences
        special_format = QTextCharFormat()
        special_format.setFontWeight(QFont.Weight.Bold)
        special_format.setBackground(QColor('#555555'))
        
        for entry in self.log_history:
            # Add visual separator between entries
            if self.log_display.toPlainText():
                self.log_display.append("")
            
            cursor = self.log_display.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.End)
            
            # Reset to default format at start of each entry
            cursor.setCharFormat(default_format)
            
            parts = re.split(r'(\\[bsec])', entry)  # Split on special sequences
            
            for part in parts:
                if not part:
                    continue
                    
                if part.startswith('\\') and len(part) == 2 and part[1] in ['b', 's', 'c', 'e']:
                    # Insert special sequence with special format
                    cursor.setCharFormat(special_format)
                    cursor.insertText(part)
                    # Immediately reset to default format
                    cursor.setCharFormat(default_format)
                else:
                    # Insert normal text with default format
                    cursor.setCharFormat(default_format)
                    cursor.insertText(part)

    def update_prediction(self, text):
        # Simulate output for all history entries plus current input
        full_output = []
        current_caps = self.capslock
        
        # Process history entries
        for entry in self.log_history:
            i = 0
            while i < len(entry):
                if entry[i] == '\\' and i + 1 < len(entry):
                    cmd = entry[i+1]
                    if cmd == 'b':  # backspace
                        if full_output:
                            full_output.pop()
                    elif cmd == 's':  # space
                        full_output.append(' ')
                    elif cmd == 'c':  # capslock
                        current_caps = not current_caps
                    elif cmd == 'e':  # enter
                        full_output.append('\n')
                    i += 2
                else:
                    char = entry[i].upper() if current_caps else entry[i]
                    full_output.append(char)
                    i += 1
        
        # Process current input
        i = 0
        while i < len(text):
            if text[i] == '\\' and i + 1 < len(text):
                cmd = text[i+1]
                if cmd == 'b':  # backspace
                    if full_output:
                        full_output.pop()
                elif cmd == 's':  # space
                    full_output.append(' ')
                elif cmd == 'c':  # capslock
                    current_caps = not current_caps
                elif cmd == 'e':  # enter
                    full_output.append('\n')
                i += 2
            else:
                char = text[i].upper() if current_caps else text[i]
                full_output.append(char)
                i += 1
                
        self.predict_label.setPlainText(''.join(full_output))

    def update_prediction_from_history(self):
        # Just call update_prediction with empty current input
        self.update_prediction("")

    def clear_logs(self):
        self.log_history = []
        self.log_display.clear()
        self.predict_label.clear()

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
                min-width: 120px;
            }
            QPushButton:hover {
                background-color: #555;
                border: 2px solid #666;
            }
            QLineEdit {
                background-color: #3a3a3a;
                border: 1px solid #555;
                border-radius: 4px;
                padding: 8px;
                color: #ddd;
            }
            QLabel {
                padding: 5px;
            }
            QFrame {
                border-radius: 5px;
                background-color: #3a3a3a;
                border: 2px solid #555;
            }
            QTextEdit {
                background-color: #3a3a3a;
                border: 1px solid #555;
                border-radius: 4px;
                padding: 8px;
                color: #ddd;
            }
        """)