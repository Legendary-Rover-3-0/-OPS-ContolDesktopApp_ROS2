import tkinter as tk
from PIL import Image, ImageTk, ImageDraw
import tkintermapview
import os
import threading
import sqlite3
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32
import rclpy
from geopy.distance import geodesic
from datetime import datetime
import math

rover_coords = {'x': 39.2012129, 'y': -111.665272}
targets = []

class RadiationMapNode(Node):
    def __init__(self):
        super().__init__('radiation_map_node')
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(Float32, 'Radiation_Publisher', self.radiation_callback, 10)
        self.latest_lat = None
        self.latest_lon = None
        self.latest_radiation = None
        self.init_database()

    def init_database(self):
        conn = sqlite3.connect('GPS/radiation_data.db')
        cursor = conn.cursor()
        cursor.execute('''
        CREATE TABLE IF NOT EXISTS radiation (
            latitude REAL,
            longitude REAL,
            radiation REAL,
            timestamp TEXT
        )''')
        conn.commit()
        conn.close()


    def gps_callback(self, msg):
        rover_coords['x'] = msg.latitude
        rover_coords['y'] = msg.longitude
        self.latest_lat = msg.latitude
        self.latest_lon = msg.longitude
        self.add_point_to_db()

    def radiation_callback(self, msg):
        self.latest_radiation = msg.data
        self.add_point_to_db()

    def add_point_to_db(self):
        if self.latest_lat and self.latest_lon and self.latest_radiation is not None:
            conn = sqlite3.connect('GPS/radiation_data.db')
            cursor = conn.cursor()
            cursor.execute('SELECT latitude, longitude FROM radiation')
            rows = cursor.fetchall()
            for lat, lon in rows:
                if geodesic((lat, lon), (self.latest_lat, self.latest_lon)).meters < 10:
                    cursor.execute('''
                        UPDATE radiation SET radiation = ?, timestamp = ?
                        WHERE latitude = ? AND longitude = ?
                    ''', (self.latest_radiation, datetime.now().strftime("%Y-%m-%d %H:%M:%S"), lat, lon))
                    conn.commit()
                    conn.close()
                    return

            cursor.execute('''
                INSERT INTO radiation (latitude, longitude, radiation, timestamp)
                VALUES (?, ?, ?, ?)''',
                (self.latest_lat, self.latest_lon, self.latest_radiation, datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
            conn.commit()
            conn.close()

def get_color_for_value(value, min_val=0.0, max_val=2.0):
    ratio = min(max((value - min_val) / (max_val - min_val), 0.0), 1.0)
    if ratio < 0.5:
        r = int(255 * (ratio * 2))
        g = 255
    else:
        r = 255
        g = int(255 * (1 - (ratio - 0.5) * 2))
    b = 0
    return (r, g, b)

def generate_colored_dot(color, size=12):
    img = Image.new("RGBA", (size, size), (255, 255, 255, 0))
    draw = ImageDraw.Draw(img)
    draw.ellipse((0, 0, size - 1, size - 1), fill=color)
    return ImageTk.PhotoImage(img)

def create_radiation_legend(min_val, max_val, width=300, height=20):
    image = Image.new("RGB", (width, height + 20), (255, 255, 255))
    draw = ImageDraw.Draw(image)
    for x in range(width):
        ratio = x / (width - 1)
        val = min_val + (max_val - min_val) * ratio
        color = get_color_for_value(val, min_val, max_val)
        draw.line([(x, 0), (x, height)], fill=color)
    draw.text((0, height), f"{min_val:.1f}", fill="black")
    draw.text((width // 2 - 10, height), f"{(min_val + max_val)/2:.1f}", fill="black")
    draw.text((width - 30, height), f"{max_val:.1f}", fill="black")
    return ImageTk.PhotoImage(image)

def center_map_on_rover(map_widget):
    global rover_coords
    # Center map on rover's current position and zoom in
    map_widget.set_position(rover_coords['x'], rover_coords['y'])
    map_widget.set_zoom(20)  # Set zoom level to max

def load_targets_from_db(map_widget):
    try:
        conn = sqlite3.connect('GPS/gps_targets.db')
        cursor = conn.cursor()
        cursor.execute('SELECT latitude, longitude FROM targets')
        rows = cursor.fetchall()
        conn.close()

        for i, (lat, lon) in enumerate(rows, start=1):
            map_widget.set_marker(lat, lon, text=f"Target {i}")
    except Exception as e:
        print(f"Błąd wczytywania targetów: {e}")

def main():
    rclpy.init()
    node = RadiationMapNode()

    root = tk.Tk()
    root.title('Radiation Map')
    root.geometry("900x900")
    root.config(bg="#f0f0f0")

    # Konfiguracja siatki
    root.rowconfigure(1, weight=1)
    root.columnconfigure(0, weight=1)

    # Przyciski na górze
    top_frame = tk.Frame(root, bg="#f0f0f0")
    top_frame.grid(row=0, column=0, sticky="ew")

    # Przycisk centrowania
    center_rover_button = tk.Button(top_frame, text="Center Map on Rover", font=("Arial", 12), command=lambda: center_map_on_rover(map_widget))
    center_rover_button.pack(pady=10)

    # Mapa
    frame = tk.Frame(root, bg="#f0f0f0")
    frame.grid(row=1, column=0, sticky="nsew")

    database_path = os.path.join('.', "GPS/mapa.db")
    map_widget = tkintermapview.TkinterMapView(frame, corner_radius=0, database_path=database_path)
    map_widget.pack(fill="both", expand=True)

    map_widget.set_position(rover_coords['x'], rover_coords['y'])
    map_widget.set_zoom(17)


    icon_img = Image.open("newRover.png").resize((50, 50))
    icon = ImageTk.PhotoImage(icon_img)
    rover_marker = map_widget.set_marker(rover_coords['x'], rover_coords['y'], text="Rover", icon=icon)

    show_labels_var = tk.BooleanVar(value=True)
    show_labels_checkbox = tk.Checkbutton(root, text="Show Labels", variable=show_labels_var, bg="#f0f0f0")
    show_labels_checkbox.grid(row=0, column=0, sticky="e", padx=10, pady=5)

    icon_cache = {}

    def load_radiation_from_db():
        try:
            conn = sqlite3.connect('GPS/radiation_data.db')
            cursor = conn.cursor()
            cursor.execute('SELECT latitude, longitude, radiation FROM radiation')
            rows = cursor.fetchall()
            conn.close()

            if not rows:
                root.after(1000, load_radiation_from_db)
                return

            values = [r[2] for r in rows]
            min_val, max_val = min(values), max(values)

            legend_photo = create_radiation_legend(min_val, max_val)
            legend_label = tk.Label(root, image=legend_photo, bg="#f0f0f0")
            legend_label.image = legend_photo
            legend_label.grid(row=0, column=0, sticky="w", padx=10, pady=5)

            map_widget.delete_all_marker()
            map_widget.set_marker(rover_coords['x'], rover_coords['y'], text="Rover", icon=icon)
            load_targets_from_db(map_widget)

            for lat, lon, value in rows:
                rounded_value = round(value, 2)
                if rounded_value not in icon_cache:
                    color = get_color_for_value(rounded_value, min_val, max_val)
                    icon_cache[rounded_value] = generate_colored_dot(color)
                icon_img = icon_cache[rounded_value]
                label = f"{rounded_value:.2f}" if show_labels_var.get() else ""
                map_widget.set_marker(lat, lon, text=label, icon=icon_img)
        except Exception as e:
            print(f"Błąd: {e}")

        root.after(1000, load_radiation_from_db)

    load_radiation_from_db()

    

    def ros_spin():
        rclpy.spin(node)

    threading.Thread(target=ros_spin, daemon=True).start()
    root.mainloop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
