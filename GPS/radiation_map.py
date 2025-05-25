import tkinter as tk
from PIL import Image, ImageTk
import tkintermapview
import os
import threading
import time
import sqlite3
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import rclpy
from geopy.distance import geodesic
from datetime import datetime
# import random

# def generate_random_gps(start_lat, start_lon, end_lat, end_lon):
#     lat = random.uniform(start_lat, end_lat)
#     lon = random.uniform(start_lon, end_lon)
#     return lat, lon

# def simulate_gps_and_radiation(node):
#     start_lat = 52 + 10/60 + 23.1/3600
#     start_lon = 20 + 53/60 + 51.1/3600
#     end_lat = 52 + 18/60 + 56.5/3600
#     end_lon = 21 + 7/60 + 22.8/3600

#     while rclpy.ok():
#         lat, lon = generate_random_gps(start_lat, start_lon, end_lat, end_lon)
#         node.latest_lat = lat
#         node.latest_lon = lon
#         node.latest_radiation = random.uniform(0, 200)
#         node.add_point_to_db()
#         time.sleep(1)


rover_coords = {'x': 52.173, 'y': 21.0}  # Domyślne współrzędne łazika


class RadiationMapNode(Node):
    def __init__(self):
        super().__init__('radiation_map_node')
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(Float32, '/Radiation_Publisher', self.radiation_callback, 10)
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
        )
        ''')
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
            # Sprawdź czy punkt już istnieje blisko w bazie
            conn = sqlite3.connect('GPS/radiation_data.db')
            cursor = conn.cursor()
            cursor.execute('SELECT latitude, longitude FROM radiation')
            rows = cursor.fetchall()
            for lat, lon in rows:
                if geodesic((lat, lon), (self.latest_lat, self.latest_lon)).meters < 10:
                    # Aktualizuj istniejący punkt
                    cursor.execute('''
                        UPDATE radiation
                        SET radiation = ?, timestamp = ?
                        WHERE latitude = ? AND longitude = ?
                    ''', (self.latest_radiation, datetime.now().strftime("%Y-%m-%d %H:%M:%S"), lat, lon))
                    conn.commit()
                    conn.close()
                    return

            # Dodaj nowy punkt
            cursor.execute('''
                INSERT INTO radiation (latitude, longitude, radiation, timestamp)
                VALUES (?, ?, ?, ?)
            ''', (self.latest_lat, self.latest_lon, self.latest_radiation, datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
            conn.commit()
            conn.close()

def main():

    def center_map_on_rover():
        map_widget.set_position(rover_coords['x'], rover_coords['y'])
        map_widget.set_zoom(20)  # Maksymalne przybliżenie

    rclpy.init()
    node = RadiationMapNode()

    root = tk.Tk()
    root.title('Radiation Map')
    root.geometry("900x900")
    root.config(bg="#f0f0f0")

    my_label = tk.LabelFrame(root, text="Radiation Map View", font=("Arial", 14), bg="#f0f0f0")
    my_label.pack(pady=20)

    database_path = os.path.join('.', "GPS/mapa.db")
    map_widget = tkintermapview.TkinterMapView(my_label, width=800, height=600, corner_radius=0, database_path=database_path)
    map_widget.set_position(rover_coords['x'], rover_coords['y'])
    map_widget.set_zoom(17)
    map_widget.pack()

    center_button = tk.Button(root, text="Center on Rover", font=("Arial", 12), command=center_map_on_rover)
    center_button.pack(pady=10)


    # Ikony
    good_icon = ImageTk.PhotoImage(Image.open("good.png").resize((25, 25)))
    medium_icon = ImageTk.PhotoImage(Image.open("medium.png").resize((25, 25)))
    bad_icon = ImageTk.PhotoImage(Image.open("bad.png").resize((25, 25)))

    rover_marker = map_widget.set_marker(rover_coords['x'], rover_coords['y'], text="Rover")

    def load_radiation_from_db():
        try:
            conn = sqlite3.connect('GPS/radiation_data.db')
            cursor = conn.cursor()
            cursor.execute('SELECT latitude, longitude, radiation FROM radiation')
            rows = cursor.fetchall()
            conn.close()

            map_widget.delete_all_marker()
            map_widget.set_marker(rover_coords['x'], rover_coords['y'], text="Rover")

            for lat, lon, value in rows:
                if value < 1:
                    icon = good_icon
                elif 1 <= value < 20:
                    icon = medium_icon
                else:
                    icon = bad_icon
                map_widget.set_marker(lat, lon, text=f"☢ {value:.2f}", icon=icon)

        except sqlite3.Error as e:
            print(f"Błąd odczytu bazy danych: {e}")

        root.after(1000, load_radiation_from_db)

    load_radiation_from_db()

    def ros_spin():
        rclpy.spin(node)

    threading.Thread(target=ros_spin, daemon=True).start()
    # Start symulacji danych GPS i radiacyjnych
    # threading.Thread(target=simulate_gps_and_radiation, args=(node,), daemon=True).start()

    root.mainloop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
