import tkinter as tk
from PIL import Image, ImageTk
import tkintermapview
import os
import threading
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import rclpy
import sqlite3
from datetime import datetime

# Global variable for marker coordinates
rover_coords = {'x': 50.01817, 'y': 21.98669}
targets = []  # Lista wszystkich targetów

class GPSTab(Node):
    def __init__(self):
        super().__init__('gps_tab')
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        
        # Initial setup
        self.root = tk.Tk()
        self.root.title('Legendary')
        self.root.geometry("900x900")
        self.root.config(bg="#f0f0f0")

        # Frame for the map
        my_label = tk.LabelFrame(self.root, text="Map View", font=("Arial", 14), bg="#f0f0f0")
        my_label.pack(pady=20)

        database_path = os.path.join('.', "GPS/mapa.db")
        self.map_widget = tkintermapview.TkinterMapView(my_label, width=800, height=600, corner_radius=0, database_path=database_path)
        self.map_widget.set_position(rover_coords['x'], rover_coords['y'])
        self.map_widget.set_zoom(18)
        self.map_widget.pack()

        # Path to the marker icon
        icon_path = "GPS/lazik.png"
        img = Image.open(icon_path)
        img = img.resize((80, 50))
        self.icon = ImageTk.PhotoImage(img)

        # Initial markers
        self.rover_marker = self.map_widget.set_marker(rover_coords['x'], rover_coords['y'], text="Rover", icon=self.icon)
        self.target_markers = []  # Lista markerów targetów

        # Labels for rover position
        self.position_label = tk.Label(self.root, font=("Arial", 12), bg="#f0f0f0")
        self.position_label.config(text=f"Rover position: {rover_coords.get('x', 'N/A')}, {rover_coords.get('y', 'N/A')}")
        self.position_label.pack(pady=10)

        # Etykieta dla targetów
        self.targets_label = tk.Label(self.root, font=("Arial", 12), bg="#f0f0f0", text="No targets")
        self.targets_label.pack(pady=5)

        # Frame for target position input
        input_frame = tk.Frame(self.root, bg="#f0f0f0")
        input_frame.pack(pady=10)

        # Labels for the input fields
        self.lat_label = tk.Label(input_frame, text="Enter Latitude for Target:", font=("Arial", 12), bg="#f0f0f0")
        self.lat_label.pack(side="left", padx=10)
        self.lat_entry = tk.Entry(input_frame, font=("Arial", 12))
        self.lat_entry.pack(side="left", padx=10)

        self.lon_label = tk.Label(input_frame, text="Enter Longitude for Target:", font=("Arial", 12), bg="#f0f0f0")
        self.lon_label.pack(side="left", padx=10)
        self.lon_entry = tk.Entry(input_frame, font=("Arial", 12))
        self.lon_entry.pack(side="left", padx=10)

        # Button to set the target from the input
        self.set_target_button = tk.Button(self.root, text="Add Target", font=("Arial", 12), command=self.set_target_from_entry)
        self.set_target_button.pack(pady=10)

        # Button to center map on rover position
        self.center_rover_button = tk.Button(self.root, text="Center Map on Rover", font=("Arial", 12), command=self.center_map_on_rover)
        self.center_rover_button.pack(pady=10)

        # Button to clear all targets
        self.clear_targets_button = tk.Button(self.root, text="Clear All Targets", font=("Arial", 12), command=self.clear_all_targets)
        self.clear_targets_button.pack(pady=10)

        # Add right-click menu options for the map
        self.map_widget.add_right_click_menu_command(label="Add Target", command=self.add_target_at_position, pass_coords=True)
        # self.map_widget.add_right_click_menu_command(label="Move Rover", command=self.move_rover, pass_coords=True)
        # self.map_widget.add_right_click_menu_command(label="Simulate", command=self.simulate_movement, pass_coords=True)

        # Inicjalizacja bazy danych
        self.init_database()
        
        # Wczytanie istniejących targetów z bazy danych
        self.load_targets_from_db()

        # Start the periodic marker update
        self.update_rover_position()

        # Start ROS2 in a separate thread
        ros_thread = threading.Thread(target=self.start_ros)
        ros_thread.daemon = True
        ros_thread.start()

        self.check_database_for_changes()

        self.root.mainloop()

    def init_database(self):
        """Inicjalizuje bazę danych, tworzy tabelę jeśli nie istnieje"""
        conn = sqlite3.connect('GPS/gps_targets.db')
        cursor = conn.cursor()
        cursor.execute('''
        CREATE TABLE IF NOT EXISTS targets (
            latitude REAL,
            longitude REAL,
            timestamp TEXT
        )
        ''')
        conn.commit()
        conn.close()

    def load_targets_from_db(self):
        """Ładuje targety z bazy danych"""
        try:
            conn = sqlite3.connect('GPS/gps_targets.db')
            cursor = conn.cursor()
            cursor.execute('SELECT latitude, longitude, timestamp FROM targets')
            rows = cursor.fetchall()
            conn.close()

            # Usuń wszystkie istniejące markery targetów
            for marker in self.target_markers:
                marker.delete()
            self.target_markers = []
            
            # Wyczyść globalną listę targetów
            global targets
            targets = []

            # Dodaj targety z bazy danych
            for row in rows:
                lat, lon, timestamp = row
                targets.append({'x': lat, 'y': lon, 'timestamp': timestamp})
            
            # Aktualizuj markery i etykiety
            self.update_target_markers()
            self.update_targets_label()

        except sqlite3.Error as e:
            print(f"Błąd podczas ładowania targetów z bazy danych: {e}")

    def check_database_for_changes(self):
        """Sprawdza, czy baza danych została zmodyfikowana i aktualizuje targety"""
        global targets
        try:
            conn = sqlite3.connect('GPS/gps_targets.db')
            cursor = conn.cursor()
            cursor.execute("SELECT COUNT(*) FROM targets")
            current_count = cursor.fetchone()[0]
            if current_count != targets:
                self.load_targets_from_db()
            conn.close()
            
            # Sprawdzaj zmiany co 1 sekundę
            self.root.after(1000, self.check_database_for_changes)

        except Exception as e:
            #print(f"Błąd podczas sprawdzania zmian w bazie danych: {e}")
            # Ponów próbę za 1 sekundę mimo błędu
            self.root.after(1000, self.check_database_for_changes)

    def add_target_to_db(self, lat, lon):
        """Dodaje target do bazy danych"""
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            conn = sqlite3.connect('GPS/gps_targets.db')
            cursor = conn.cursor()
            cursor.execute('''
            INSERT INTO targets (latitude, longitude, timestamp)
            VALUES (?, ?, ?)
            ''', (lat, lon, timestamp))
            conn.commit()
            conn.close()
            return timestamp
        except sqlite3.Error as e:
            print(f"Błąd podczas dodawania targetu do bazy danych: {e}")
            return None

    def clear_targets_from_db(self):
        """Usuwa wszystkie targety z bazy danych"""
        try:
            conn = sqlite3.connect('GPS/gps_targets.db')
            cursor = conn.cursor()
            cursor.execute('DELETE FROM targets')
            conn.commit()
            conn.close()
        except sqlite3.Error as e:
            print(f"Błąd podczas usuwania targetów z bazy danych: {e}")

    def start_ros(self):
        rclpy.spin(self)  # Start the ROS2 spin loop in a separate thread

    def gps_callback(self, msg: NavSatFix):
        """Handle new GPS data"""
        rover_coords['x'] = msg.latitude
        rover_coords['y'] = msg.longitude
        # Update the position label in the main Tkinter thread
        self.root.after(0, self.update_position_label)

    def update_rover_position(self):
        global rover_coords, targets
        # Update the rover marker position
        self.rover_marker.set_position(rover_coords['x'], rover_coords['y'])
        
        # Update position label
        self.update_position_label()
        
        # Clear the path
        self.map_widget.delete_all_path()
        
        # Draw paths between rover and targets
        if targets:
            # First path from rover to first target
            positions = [self.rover_marker.position]
            
            # Add all target positions
            for marker in self.target_markers:
                positions.append(marker.position)
            
            # Create paths
            for i in range(len(positions) - 1):
                self.map_widget.set_path([positions[i], positions[i+1]])
        
        # Schedule the next update
        self.root.after(500, self.update_rover_position)

    def update_position_label(self):
        """Update position label with new coordinates."""
        self.position_label.config(text=f"Rover position: {rover_coords['x']}, {rover_coords['y']}")

    def update_targets_label(self):
        """Update targets label with information about targets."""
        global targets
        if not targets:
            self.targets_label.config(text="No targets")
        else:
            targets_text = f"Targets ({len(targets)}):"
            self.targets_label.config(text=targets_text)

    def update_target_markers(self):
        """Updates target markers based on targets list"""
        # Usuń wszystkie istniejące markery
        for marker in self.target_markers:
            marker.delete()
        self.target_markers = []

        # Dodaj markery dla wszystkich targetów
        for i, target in enumerate(targets):
            marker = self.map_widget.set_marker(
                target['x'], 
                target['y'], 
                text=f"Target {i+1}"
            )
            self.target_markers.append(marker)

    def simulate_movement(self, coords=None):
        global rover_coords, targets
        if not targets:
            return

        # Calculate movement vector to the first target
        dx = targets[0]['x'] - rover_coords['x']
        dy = targets[0]['y'] - rover_coords['y']
        
        # Calculate distance to target
        distance = (dx ** 2 + dy ** 2) ** 0.5
        
        # Determine movement step based on speed
        speed = 0.00002  # Adjust this value for desired speed
        if distance > speed:
            # Normalize movement vector
            dx /= distance
            dy /= distance
            
            # Update rover position
            rover_coords['x'] += dx * speed
            rover_coords['y'] += dy * speed
        else:
            # If we've reached the target, remove it
            if targets:
                targets.pop(0)
                
                # Remove target from database
                conn = sqlite3.connect('GPS/gps_targets.db')
                cursor = conn.cursor()
                cursor.execute('DELETE FROM targets ORDER BY ROWID ASC LIMIT 1')
                conn.commit()
                conn.close()
                
                # Update markers
                self.update_target_markers()
                self.update_targets_label()
        
        self.root.after(50, self.simulate_movement)

    def move_rover(self, coords):
        global rover_coords
        rover_coords['x'] = coords[0]
        rover_coords['y'] = coords[1]

    def add_target_at_position(self, coords):
        """Dodaje nowy target na mapie z podanych współrzędnych"""
        global targets
        lat, lon = coords[0], coords[1]
        
        # Dodaj do bazy danych i pobierz timestamp
        timestamp = self.add_target_to_db(lat, lon)
        if timestamp:
            # Dodaj do listy targetów
            targets.append({'x': lat, 'y': lon, 'timestamp': timestamp})
            
            # Aktualizuj markery i etykiety
            self.update_target_markers()
            self.update_targets_label()

    def set_target_from_entry(self):
        """Dodaj target z pól wprowadzania"""
        try:
            lat = float(self.lat_entry.get())
            lon = float(self.lon_entry.get())
            self.add_target_at_position((lat, lon))
            # Wyczyść pola
            self.lat_entry.delete(0, tk.END)
            self.lon_entry.delete(0, tk.END)
        except ValueError:
            self.show_invalid_coordinates_message()

    def clear_all_targets(self):
        """Usuwa wszystkie targety"""
        global targets
        
        # Usuń wszystkie targety z bazy danych
        self.clear_targets_from_db()
        
        # Usuń wszystkie markery
        for marker in self.target_markers:
            marker.delete()
        self.target_markers = []
        
        # Wyczyść listę targetów
        targets = []
        
        # Aktualizuj etykiety
        self.update_targets_label()
        
        # Usuń ścieżki
        self.map_widget.delete_all_path()

    def show_invalid_coordinates_message(self):
        invalid_coords_label = tk.Label(self.root, text="Invalid Coordinates! Please enter valid latitude and longitude.", font=("Arial", 10), fg="red", bg="#f0f0f0")
        invalid_coords_label.pack(pady=5)
        self.root.after(3000, invalid_coords_label.destroy)  # Remove the label after 3 seconds

    def center_map_on_rover(self):
        # Center map on rover's current position and zoom in
        self.map_widget.set_position(rover_coords['x'], rover_coords['y'])
        self.map_widget.set_zoom(20)  # Set zoom level to max

if __name__ == "__main__":
    rclpy.init()
    gps_tab = GPSTab()
    rclpy.shutdown()