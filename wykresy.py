import os
import datetime
from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                            QPushButton, QTabWidget, QSpinBox, QLabel, QGroupBox)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.dates as mdates


class PlotApp(QWidget):
    def __init__(self):
        super().__init__()
        self.data_directory = "sensor_data"  # Katalog z danymi
        self.max_readings = 100  # Domyślna maksymalna liczba odczytów
        self.init_ui()
        self.setWindowTitle("Science Data Visualizer")
        self.setGeometry(100, 100, 1200, 800)
        self.time = 2

    def init_ui(self):
        main_layout = QVBoxLayout()

        # Panel kontrolny
        control_group = QGroupBox("Ustawienia wyświetlania")
        control_layout = QHBoxLayout()
        
        # Przycisk odświeżania
        refresh_button = QPushButton('Odśwież wszystkie wykresy')
        refresh_button.clicked.connect(self.refresh_all_plots)
        
        # Kontrolka do ustawienia maksymalnej liczby odczytów
        readings_label = QLabel('Maks. liczba odczytów:')
        self.readings_spinbox = QSpinBox()
        self.readings_spinbox.setMinimum(10)
        self.readings_spinbox.setMaximum(10000)
        self.readings_spinbox.setValue(self.max_readings)
        
        # Przycisk akceptacji zmian
        accept_button = QPushButton('Zaakceptuj')
        accept_button.clicked.connect(self.apply_settings)
        
        control_layout.addWidget(refresh_button)
        control_layout.addStretch()
        control_layout.addWidget(readings_label)
        control_layout.addWidget(self.readings_spinbox)
        control_layout.addWidget(accept_button)
        control_group.setLayout(control_layout)

        # Zakładki dla różnych typów wykresów
        self.tabs = QTabWidget()
        
        # Dodaj zakładki
        self.co2_tab = QWidget()
        self.methane_tab = QWidget()
        self.temp_tab = QWidget()
        self.humidity_tab = QWidget()
        self.radiation_tab = QWidget()
        
        self.tabs.addTab(self.create_plot_tab(self.co2_tab, "Concentration of CO₂"), "CO₂ [ppm]")
        self.tabs.addTab(self.create_plot_tab(self.methane_tab, "Concentration of Metan"), "Metan [ppm]")
        self.tabs.addTab(self.create_plot_tab(self.temp_tab, "Soil temperature"), "Temp. [°C]")
        self.tabs.addTab(self.create_plot_tab(self.humidity_tab, "Soil moisture"), "Moisture [%]")
        self.tabs.addTab(self.create_plot_tab(self.radiation_tab, "Radiation"), "Radiation [μSv/h]")
        
        main_layout.addWidget(control_group)
        main_layout.addWidget(self.tabs)
        self.setLayout(main_layout)
        
        # Pierwsze ładowanie danych
        self.refresh_all_plots()

    def find_newest_timestamp(self, filename):
        """Znajdź najnowszy timestamp w pliku"""
        newest = None
        try:
            with open(os.path.join(self.data_directory, filename), "r") as f:
                for line in f:
                    parts = line.strip().split(", ")
                    if len(parts) >= 2:
                        try:
                            timestamp = datetime.datetime.strptime(parts[0], "%Y-%m-%d %H:%M:%S")
                            if newest is None or timestamp > newest:
                                newest = timestamp
                        except ValueError:
                            continue
        except FileNotFoundError:
            pass
        return newest

    def apply_settings(self):
        """Zastosuj ustawienia po kliknięciu przycisku Zaakceptuj"""
        self.max_readings = self.readings_spinbox.value()
        self.refresh_all_plots()

    def create_plot_tab(self, tab, title):
        layout = QVBoxLayout(tab)
        figure = Figure()
        canvas = FigureCanvas(figure)
        layout.addWidget(canvas)
        tab.figure = figure
        tab.canvas = canvas
        tab.title = title
        return tab

    def load_data_from_file(self, filename):
        timestamps = []
        values = []
        
        # Znajdź najnowszy timestamp w pliku
        newest_timestamp = self.find_newest_timestamp(filename)
        if newest_timestamp is None:
            return timestamps, values
        
        one_hour_before_newest = newest_timestamp - datetime.timedelta(minutes=35)  ##TU ZMIENIĆ 
        
        try:
            with open(os.path.join(self.data_directory, filename), "r") as f:
                # Odczytaj wszystkie linie i odwróć kolejność (najnowsze na końcu)
                lines = f.readlines()
                
                # Ogranicz liczbę odczytów do max_readings
                lines = lines[-self.max_readings:]
                
                for line in lines:
                    parts = line.strip().split(", ")
                    if len(parts) >= 2:
                        try:
                            timestamp = datetime.datetime.strptime(parts[0], "%Y-%m-%d %H:%M:%S")
                            
                            # Pomijaj dane starsze niż godzina przed najnowszym timestampem
                            if timestamp < one_hour_before_newest:
                                continue
                                
                            # Dla plików z wieloma wartościami (CO2, metan)
                            if len(parts) > 2:
                                for i, val in enumerate(parts[1:]):
                                    if len(values) <= i:
                                        values.append([])
                                        timestamps.append([])
                                    timestamps[i].append(timestamp)
                                    values[i].append(float(val))
                            else:
                                if not values:
                                    values.append([])
                                    timestamps.append([])
                                timestamps[0].append(timestamp)
                                values[0].append(float(parts[1]))
                        except ValueError as e:
                            print(f"Błąd parsowania linii: {line}. {e}")
        except FileNotFoundError:
            print(f"Plik {filename} nie istnieje.")
        except Exception as e:
            print(f"Błąd wczytywania {filename}: {e}")
        
        return timestamps, values

    def plot_data(self, tab, filename, ylabel):
        tab.figure.clear()
        ax = tab.figure.add_subplot(111)
        
        timestamps, values = self.load_data_from_file(filename)
        
        if not timestamps:
            ax.text(0.5, 0.5, 'Brak danych', ha='center', va='center')
            tab.canvas.draw()
            return
        
        newest_timestamp = self.find_newest_timestamp(filename)
        if newest_timestamp:
            time_range = f"from {(newest_timestamp - datetime.timedelta(minutes=35)):%H:%M} to {newest_timestamp:%H:%M}" ##TU ZMIENIĆ 
        else:
            time_range = "last hour"
        
        colors = ['b', 'g', 'r', 'c', 'm', 'y']
        labels = ['Sensor1', 'Sensor 2']  # Domyślne etykiety
        
        for i in range(len(timestamps)):
            if i < len(labels):
                label = labels[i]
            else:
                label = f'Dane {i+1}'
                
            ax.plot(timestamps[i], values[i], 
                   color=colors[i % len(colors)], 
                   label=label, 
                   marker='o', 
                   markersize=3, 
                   linestyle='-', 
                   linewidth=1)
        
        # Formatowanie osi czasu
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M\n%d-%m'))
        ax.xaxis.set_major_locator(mdates.AutoDateLocator())
        
        ax.set_ylabel(ylabel)
        ax.set_title(f"{tab.title} (last {len(timestamps[0])} readings)")
        ax.legend(loc='upper right')
        ax.grid(True, linestyle='--', alpha=0.7)
        tab.figure.autofmt_xdate()
        tab.canvas.draw()

    def refresh_all_plots(self):
        # Mapowanie zakładek na odpowiednie pliki i etykiety
        plot_config = [
            (self.co2_tab, "co2.txt", "CO₂ [ppm]"),
            (self.methane_tab, "methane.txt", "Metan [ppm]"),
            (self.temp_tab, "soil_temp.txt", "Temp. [°C]"),
            (self.humidity_tab, "soil_humidity.txt", "Moisture [%]"),
            (self.radiation_tab, "radiation.txt", "Radiation [μSv/h]")
        ]
        
        for tab, filename, ylabel in plot_config:
            self.plot_data(tab, filename, ylabel)


if __name__ == "__main__":
    app = QApplication([])
    plot_app = PlotApp()
    plot_app.show()
    app.exec()