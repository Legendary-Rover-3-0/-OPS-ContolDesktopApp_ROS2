import tkinter as tk
from PIL import Image, ImageTk
import tkintermapview
import os
import threading
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import rclpy
import sqlite3
from datetime import datetime

rover_coords = {'x': 39.2012129, 'y': -111.665272, 'alt': 0.0}
targets = []

class GPSTab(Node):
    def __init__(self):
        super().__init__('gps_tab')
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(Float32, '/heading', self.heading_callback, 10)
        self.rover_yaw_deg = 0

        self.root = tk.Tk()
        self.root.title('Legendary GPS Tracker')
        self.root.geometry("1000x800")
        self.root.config(bg="#f0f0f0")

        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        # Center button at the top
        self.center_rover_button = tk.Button(self.root, text="Center Map on Rover", font=("Arial", 12), command=self.center_map_on_rover)
        self.center_rover_button.grid(row=0, column=0, pady=10)

        # Map frame
        map_frame = tk.Frame(self.root)
        map_frame.grid(row=1, column=0, sticky="nsew")

        database_path = os.path.join('.', "GPS/mapa.db")
        self.map_widget = tkintermapview.TkinterMapView(map_frame, corner_radius=0, database_path=database_path)
        self.map_widget.pack(fill="both", expand=True)
        self.map_widget.set_position(rover_coords['x'], rover_coords['y'])
        self.map_widget.set_zoom(18)

        # Rover icon
        icon_path = "rover_gps.png"
        self.original_icon_img = Image.open(icon_path).resize((50, 50))
        self.icon = ImageTk.PhotoImage(self.original_icon_img)
        self.rover_marker = self.map_widget.set_marker(rover_coords['x'], rover_coords['y'], text="Rover", icon=self.icon)

        # Inicjalizacja pustej listy markerów targetów
        self.target_markers = []

        # Labels
        self.position_label = tk.Label(self.root, font=("Arial", 12), bg="#f0f0f0")
        self.position_label.grid(row=2, column=0, pady=5)

        self.targets_label = tk.Label(self.root, font=("Arial", 12), bg="#f0f0f0", text="No targets")
        self.targets_label.grid(row=3, column=0, pady=5)

        # Clear targets button
        self.clear_targets_button = tk.Button(self.root, text="Clear All Targets", font=("Arial", 12), command=self.clear_all_targets)
        self.clear_targets_button.grid(row=4, column=0, pady=10)

        # Right click add
        self.map_widget.add_right_click_menu_command(label="Add Target", command=self.add_target_at_position, pass_coords=True)

        self.init_database()
        self.load_targets_from_db()
        self.update_rover_position()

        ros_thread = threading.Thread(target=self.start_ros)
        ros_thread.daemon = True
        ros_thread.start()

        self.check_database_for_changes()

        self.root.mainloop()

    # === Reszta metod (niezmienione lub lekko zmodyfikowane) ===

    def gps_callback(self, msg: NavSatFix):
        rover_coords['x'] = msg.latitude
        rover_coords['y'] = msg.longitude
        rover_coords['alt'] = msg.altitude
        self.root.after(0, self.update_position_label)

    def heading_callback(self, msg: Float32):
        self.rover_yaw_deg = msg.data
        #self.root.after(0, self.update_rover_icon)

    def update_position_label(self):
        self.position_label.config(text=f"Rover position: {rover_coords['x']:.6f}, {rover_coords['y']:.6f}, Altitude: {rover_coords['alt']:.2f} m")

    def center_map_on_rover(self):
        self.map_widget.set_position(rover_coords['x'], rover_coords['y'])
        self.map_widget.set_zoom(20)

    def start_ros(self):
        rclpy.spin(self)

    def update_rover_position(self):
        self.update_position_label()
        self.update_rover_icon()
        
        #self.map_widget.delete_all_path()
        # if targets:
        #     positions = [self.rover_marker.position] + [marker.position for marker in self.target_markers]
        #     for i in range(len(positions) - 1):
        #         self.map_widget.set_path([positions[i], positions[i+1]])
        self.root.after(500, self.update_rover_position)

    def update_rover_icon(self):
        rotated = self.original_icon_img.rotate(-self.rover_yaw_deg, expand=True)
        self.icon = ImageTk.PhotoImage(rotated)
        lat, lon = rover_coords['x'], rover_coords['y']
        if self.rover_marker:
            self.rover_marker.delete()
        self.rover_marker = self.map_widget.set_marker(lat, lon, text="Rover", icon=self.icon)

    def add_target_at_position(self, coords):
        lat, lon = coords[0], coords[1]
        timestamp = self.add_target_to_db(lat, lon)
        if timestamp:
            targets.append({'x': lat, 'y': lon, 'timestamp': timestamp})
            self.update_target_markers()
            self.update_targets_label()

    def clear_all_targets(self):
        global targets
        self.clear_targets_from_db()
        for marker in self.target_markers:
            marker.delete()
        self.target_markers = []
        targets = []
        self.update_targets_label()
        self.map_widget.delete_all_path()

    def update_targets_label(self):
        if not targets:
            self.targets_label.config(text="No targets")
        else:
            self.targets_label.config(text=f"Targets ({len(targets)}):")

    def update_target_markers(self):
        for marker in self.target_markers:
            marker.delete()
        self.target_markers = []
        for i, target in enumerate(targets):
            marker = self.map_widget.set_marker(target['x'], target['y'], text=f"Target {i+1}")
            self.target_markers.append(marker)

    def init_database(self):
        conn = sqlite3.connect('GPS/gps_targets.db')
        cursor = conn.cursor()
        cursor.execute('''
        CREATE TABLE IF NOT EXISTS targets (
            latitude REAL,
            longitude REAL,
            timestamp TEXT
        )''')
        conn.commit()
        conn.close()

    def load_targets_from_db(self):
        try:
            conn = sqlite3.connect('GPS/gps_targets.db')
            cursor = conn.cursor()
            cursor.execute('SELECT latitude, longitude, timestamp FROM targets')
            rows = cursor.fetchall()
            conn.close()
            for marker in self.target_markers:
                marker.delete()
            self.target_markers = []
            global targets
            targets = []
            for row in rows:
                lat, lon, timestamp = row
                targets.append({'x': lat, 'y': lon, 'timestamp': timestamp})
            self.update_target_markers()
            self.update_targets_label()
        except sqlite3.Error as e:
            print(f"Błąd podczas ładowania targetów: {e}")

    def add_target_to_db(self, lat, lon):
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
            print(f"Błąd dodawania targetu: {e}")
            return None

    def clear_targets_from_db(self):
        try:
            conn = sqlite3.connect('GPS/gps_targets.db')
            cursor = conn.cursor()
            cursor.execute('DELETE FROM targets')
            conn.commit()
            conn.close()
        except sqlite3.Error as e:
            print(f"Błąd usuwania targetów: {e}")

    def check_database_for_changes(self):
        global targets
        try:
            conn = sqlite3.connect('GPS/gps_targets.db')
            cursor = conn.cursor()
            cursor.execute("SELECT COUNT(*) FROM targets")
            current_count = cursor.fetchone()[0]
            if current_count != len(targets):
                self.load_targets_from_db()
            conn.close()
            self.root.after(1000, self.check_database_for_changes)
        except:
            self.root.after(1000, self.check_database_for_changes)

if __name__ == "__main__":
    rclpy.init()
    gps_tab = GPSTab()
    rclpy.shutdown()
