import tkinter as tk
from PIL import Image, ImageTk
import tkintermapview
import os
import threading
import random
import time
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import rclpy
from geopy.distance import geodesic

class RadiationMapNode(Node):
    def __init__(self):
        super().__init__('radiation_map_node')
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(Float32, '/Radiation_Publisher', self.radiation_callback, 10)
        self.latest_lat = None
        self.latest_lon = None
        self.latest_radiation = None
        self.radiation_points = []  # List of (lat, lon, value)

    def gps_callback(self, msg):
        self.latest_lat = msg.latitude
        self.latest_lon = msg.longitude
        self.add_point()

    def radiation_callback(self, msg):
        self.latest_radiation = msg.data
        self.add_point()

    def add_point(self):
        if self.latest_lat is not None and self.latest_lon is not None and self.latest_radiation is not None:
            # Check if a point already exists nearby
            for i, (lat, lon, _) in enumerate(self.radiation_points):
                distance = geodesic((lat, lon), (self.latest_lat, self.latest_lon)).meters
                if distance < 10: #SET DISTANCE
                    self.radiation_points[i] = (lat, lon, self.latest_radiation)
                    return
            # If no point is found nearby, add a new point
            self.radiation_points.append((self.latest_lat, self.latest_lon, self.latest_radiation))

def generate_random_gps(start_lat, start_lon, end_lat, end_lon):
    # Generating a random latitude between the start and end latitude
    lat = random.uniform(start_lat, end_lat)
    # Generating a random longitude between the start and end longitude
    lon = random.uniform(start_lon, end_lon)
    return lat, lon

def simulate_gps_and_radiation(node):
    start_lat = 52 + 10/60 + 23.1/3600
    start_lon = 20 + 53/60 + 51.1/3600
    end_lat = 52 + 18/60 + 56.5/3600
    end_lon = 21 + 7/60 + 22.8/3600

    while rclpy.ok():
        lat, lon = generate_random_gps(start_lat, start_lon, end_lat, end_lon)
        node.latest_lat = lat
        node.latest_lon = lon
        
        node.add_point()
        time.sleep(1)

def main():
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
    map_widget.set_zoom(18)
    map_widget.pack()

    # Function to update radiation markers
    def update_radiation_markers():
        map_widget.delete_all_marker()
        for lat, lon, value in node.radiation_points:
            # Color/size can be adjusted based on value
            map_widget.set_marker(lat, lon, text=f"☢ {value:.2f}")
        root.after(1000, update_radiation_markers)

    update_radiation_markers()

    def ros_spin():
        rclpy.spin(node)

    # Start the ROS simulation in a separate thread
    threading.Thread(target=ros_spin, daemon=True).start()

    # Start simulating GPS and radiation data in a separate thread
    #threading.Thread(target=simulate_gps_and_radiation, args=(node,), daemon=True).start()

    root.mainloop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
