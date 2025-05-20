from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel
from PyQt6.QtGui import QFont
from PyQt6.QtCore import QProcess
import os

class RadiationMapTab(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        label = QLabel("Radiation Map (opens in separate window)")
        label.setFont(QFont("Arial", 14))
        layout.addWidget(label)

        open_btn = QPushButton("Open Radiation Map")
        open_btn.setFont(QFont("Arial", 14))
        open_btn.setFixedHeight(70)
        open_btn.setFixedWidth(300)
        open_btn.clicked.connect(self.open_radiation_map)
        layout.addWidget(open_btn)
        layout.addStretch()
        self.setLayout(layout)

    def open_radiation_map(self):
        # Launch the Tkinter-based radiation map window
        radiation_map_path = os.path.join(os.path.dirname(__file__), 'GPS/radiation_map.py')
        QProcess.startDetached('python3', [radiation_map_path])