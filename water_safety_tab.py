import datetime
import collections
import numpy as np
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                            QGroupBox, QSpinBox, QPushButton)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

class WaterSafetyTab(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        
        # Parameters for pH analysis
        self.current_ph = 7.0  # Neutral pH as default
        self.window_size = 2  # Default window size in seconds for averaging
        self.ph_history = collections.deque(maxlen=1000)  # Store pH readings with timestamps
        self.highest_avg_period = {"start": None, "end": None, "avg": 0.0}
        
        # Initialize UI
        self.init_ui()
        
        # Subscribe to pH sensor data
        self.node.create_subscription(Float32, '/ph_sensor', self.ph_callback, 10)
        
        # Timer for updating analysis
        self.analysis_timer = QTimer()
        self.analysis_timer.timeout.connect(self.update_analysis)
        self.analysis_timer.start(5000)  # Update every 5 seconds
        
    def init_ui(self):
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)
        
        # Current pH Display
        current_ph_group = QGroupBox("Current Water pH Status")
        current_ph_group.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        current_ph_layout = QVBoxLayout()
        
        self.ph_value_label = QLabel("Current pH: 7.0")
        self.ph_value_label.setFont(QFont('Arial', 16))
        self.ph_value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        self.safety_label = QLabel("Water Safety: Unknown")
        self.safety_label.setFont(QFont('Arial', 14, QFont.Weight.Bold))
        self.safety_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        self.explanation_label = QLabel("No pH data available yet.")
        self.explanation_label.setFont(QFont('Arial', 12))
        self.explanation_label.setWordWrap(True)
        self.explanation_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        current_ph_layout.addWidget(self.ph_value_label)
        current_ph_layout.addWidget(self.safety_label)
        current_ph_layout.addWidget(self.explanation_label)
        current_ph_group.setLayout(current_ph_layout)
        main_layout.addWidget(current_ph_group)
        
        # Analysis Settings
        analysis_settings_group = QGroupBox("Analysis Settings")
        analysis_settings_group.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        analysis_settings_layout = QHBoxLayout()
        
        window_size_label = QLabel("Window Size (seconds):")
        window_size_label.setFont(QFont('Arial', 11))
        
        self.window_size_spin = QSpinBox()
        self.window_size_spin.setRange(2, 10)  # 10 seconds to 1 hour
        self.window_size_spin.setValue(self.window_size)
        self.window_size_spin.setSingleStep(1)
        self.window_size_spin.valueChanged.connect(self.update_window_size)
        
        self.update_button = QPushButton("Update Analysis")
        self.update_button.clicked.connect(self.update_analysis)
        
        analysis_settings_layout.addWidget(window_size_label)
        analysis_settings_layout.addWidget(self.window_size_spin)
        analysis_settings_layout.addWidget(self.update_button)
        analysis_settings_group.setLayout(analysis_settings_layout)
        main_layout.addWidget(analysis_settings_group)
        
        # Highest pH Period
        highest_ph_group = QGroupBox("Highest Average pH Period")
        highest_ph_group.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        highest_ph_layout = QVBoxLayout()
        
        self.highest_ph_label = QLabel("No data available yet.")
        self.highest_ph_label.setFont(QFont('Arial', 11))
        self.highest_ph_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        highest_ph_layout.addWidget(self.highest_ph_label)
        highest_ph_group.setLayout(highest_ph_layout)
        main_layout.addWidget(highest_ph_group)
        
        # pH History Graph
        graph_group = QGroupBox("pH History")
        graph_group.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        graph_layout = QVBoxLayout()
        
        self.figure = plt.figure(figsize=(5, 3), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        graph_layout.addWidget(self.canvas)
        
        graph_group.setLayout(graph_layout)
        main_layout.addWidget(graph_group)
        
        self.setLayout(main_layout)
        
        # Apply dark theme styling
        self.apply_styles()
        
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
            }
            QPushButton:hover {
                background-color: #555;
                border: 2px solid #666;
            }
            QLabel {
                padding: 5px;
            }
            QSpinBox {
                background-color: #3a3a3a;
                border: 1px solid #555;
                color: white;
                padding: 5px;
                border-radius: 3px;
            }
        """)
        
    def ph_callback(self, msg: Float32):
        """Process new pH sensor readings"""
        self.current_ph = msg.data
        timestamp = datetime.datetime.now()
        self.ph_history.append((timestamp, self.current_ph))
        
        # Update the display
        self.update_display()
        
    def update_display(self):
        """Update the UI with current pH and safety status"""
        # Update pH value
        self.ph_value_label.setText(f"Current pH: {self.current_ph:.2f}")
        
        # Determine water safety
        safety_status, color, explanation = self.evaluate_water_safety(self.current_ph)
        
        self.safety_label.setText(f"Water Safety: {safety_status}")
        self.safety_label.setStyleSheet(f"color: {color};")
        self.explanation_label.setText(explanation)
        
        # Update the graph
        self.update_graph()
        
    def update_graph(self):
        """Update the pH history graph"""
        self.figure.clear()
        ax = self.figure.add_subplot(111)
        
        if len(self.ph_history) > 0:
            times, ph_values = zip(*[(t, p) for t, p in self.ph_history])
            
            # Convert datetime to relative seconds for plotting
            seconds = [(t - times[0]).total_seconds() for t in times]
            
            ax.plot(seconds, ph_values, 'b-')
            ax.set_xlabel('Time (seconds)')
            ax.set_ylabel('pH Value')
            ax.grid(True)
            
            # Add safety range markers
            ax.axhspan(6.5, 8.5, alpha=0.2, color='green', label='Safe Range')
            ax.axhline(y=6.5, color='green', linestyle='--')
            ax.axhline(y=8.5, color='green', linestyle='--')
            
            ax.set_title('pH History')
            ax.legend()

            # Set dark background for the plot
            ax.set_facecolor('#2d2d2d')
            self.figure.set_facecolor('#333333')
            ax.tick_params(colors='#dddddd')
            ax.xaxis.label.set_color('#dddddd')
            ax.yaxis.label.set_color('#dddddd')
            ax.title.set_color('#dddddd')
        
        self.canvas.draw()
        
    def evaluate_water_safety(self, ph):
        """Evaluate water safety based on pH value"""
        if ph < 4.0:
            return "Extremely Unsafe", "red", "This water is extremely acidic and unsafe to drink. It may cause severe damage to your digestive system."
        elif 4.0 <= ph < 6.5:
            return "Unsafe", "orange", "This water is too acidic for drinking. It may cause digestive discomfort and is not recommended."
        elif 6.5 <= ph <= 8.5:
            return "Safe", "green", "This water has an optimal pH balance and is safe for drinking."
        elif 8.5 < ph <= 10.0:
            return "Unsafe", "orange", "This water is too alkaline for drinking. It may taste bitter and could cause skin and eye irritation."
        else:  # pH > 10
            return "Extremely Unsafe", "red", "This water is extremely alkaline and unsafe to drink. It can cause severe irritation and damage to organs."
            
    def update_window_size(self, value):
        """Update the window size for analysis"""
        self.window_size = value
        
    def update_analysis(self):
        """Update analysis of pH history"""
        if len(self.ph_history) < 2:
            return
            
        # Find highest average pH period
        highest_avg = 0.0
        highest_start = None
        highest_end = None
        
        # Get current time
        current_time = datetime.datetime.now()
        
        # Analyze different starting points
        for i in range(len(self.ph_history)):
            start_time = self.ph_history[i][0]
            
            # If this starting point is already beyond our window, skip
            if (current_time - start_time).total_seconds() > self.window_size:
                continue
                
            # Find all readings within the window from this starting point
            window_readings = []
            for j in range(i, len(self.ph_history)):
                if (self.ph_history[j][0] - start_time).total_seconds() <= self.window_size:
                    window_readings.append(self.ph_history[j][1])
                else:
                    break
                    
            if window_readings:
                avg = sum(window_readings) / len(window_readings)
                if avg > highest_avg:
                    highest_avg = avg
                    highest_start = start_time
                    highest_end = self.ph_history[j-1][0] if j > i else start_time
        
        # Update the highest average period
        if highest_start and highest_end:
            self.highest_avg_period = {
                "start": highest_start,
                "end": highest_end,
                "avg": highest_avg
            }
            
            start_str = highest_start.strftime("%H:%M:%S")
            end_str = highest_end.strftime("%H:%M:%S")
            
            self.highest_ph_label.setText(
                f"Highest average pH of {highest_avg:.2f} occurred between\n"
                f"{start_str} and {end_str}\n"
                f"({(highest_end - highest_start).total_seconds():.0f} seconds)"
            )