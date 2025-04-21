import tkinter as tk
from PIL import Image, ImageTk
import tkintermapview
import os
import threading
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import rclpy

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
            self.radiation_points.append((self.latest_lat, self.latest_lon, self.latest_radiation))

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

    threading.Thread(target=ros_spin, daemon=True).start()
    root.mainloop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()