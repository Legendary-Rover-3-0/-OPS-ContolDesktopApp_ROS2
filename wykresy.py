import os
import datetime
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt


class PlotApp(QWidget):
    def __init__(self):
        super().__init__()
        self.data_directory = "sensor_data"  # Katalog z danymi
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Sensor Data Plotter")
        self.setGeometry(100, 100, 800, 600)

        main_layout = QVBoxLayout()

        # Przyciski do wyboru typu wykresu
        self.plot_buttons = QHBoxLayout()
        self.temperature_plot_button = QPushButton('Temperature Plot')
        self.mass_plot_button = QPushButton('Mass Plot')
        self.soil_moisture_plot_button = QPushButton('Soil Moisture Plot')

        self.temperature_plot_button.clicked.connect(lambda: self.plot_data('temperature'))
        self.mass_plot_button.clicked.connect(lambda: self.plot_data('mass'))
        self.soil_moisture_plot_button.clicked.connect(lambda: self.plot_data('soil_moisture'))

        self.plot_buttons.addWidget(self.temperature_plot_button)
        self.plot_buttons.addWidget(self.mass_plot_button)
        self.plot_buttons.addWidget(self.soil_moisture_plot_button)

        # Canvas do wyświetlania wykresów
        self.plot_figure = Figure()
        self.plot_canvas = FigureCanvas(self.plot_figure)
        
        main_layout.addLayout(self.plot_buttons)
        main_layout.addWidget(self.plot_canvas)

        self.setLayout(main_layout)

    def load_sensor_data(self, file_pattern):
        data = []
        for i in range(1, 5):  # Dla 4 sensorów
            file_path = os.path.join(self.data_directory, file_pattern.format(i))
            sensor_data = []
            try:
                with open(file_path, "r") as f:
                    lines = f.readlines()[-100:]  # Pobierz ostatnie 100 linii
                    for line in lines:
                        parts = line.strip().split(", ")
                        if len(parts) != 2:
                            continue
                        value = float(parts[1])
                        sensor_data.append(value)
            except FileNotFoundError:
                print(f"Plik {file_path} nie istnieje.")
            except Exception as e:
                print(f"Błąd wczytywania {file_path}: {e}")
            
            data.append(sensor_data)
        
        return data

    def plot_data(self, plot_type):
        self.plot_figure.clear()
        ax = self.plot_figure.add_subplot(111)
        
        file_patterns = {
            'temperature': "temperature_sensor_{}.txt",
            'mass': "mass_sensor_{}.txt",
            'soil_moisture': "soil_moisture_sensor_{}.txt"
        }
        
        if plot_type in file_patterns:
            data_history = self.load_sensor_data(file_patterns[plot_type])
            ylabel_dict = {
                'temperature': "Temperature (°C)",
                'mass': "Mass (g)",
                'soil_moisture': "Soil Moisture"
            }
            ylabel = ylabel_dict[plot_type]
        else:
            return
        
        colors = ['r', 'g', 'b', 'm']  # Kolory dla każdego sensora
        for i in range(4):
            ax.plot(range(len(data_history[i])), data_history[i], color=colors[i], label=f'Sensor {i+1}')
        
        ax.set_ylabel(ylabel)
        ax.set_xlabel("Time Steps")
        ax.legend(loc='upper right')
        ax.grid(True)
        self.plot_canvas.draw()


if __name__ == "__main__":
    app = QApplication([])
    plot_app = PlotApp()
    plot_app.show()
    app.exec()