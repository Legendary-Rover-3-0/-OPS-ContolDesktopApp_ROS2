import tkinter as tk
from PIL import Image, ImageTk
import tkintermapview
import os
import threading
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import rclpy

# Global variable for marker coordinates
rover_coords = {'x': 50.01817, 'y': 21.98669}
target_coords = {'x': None, 'y': None}

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
        self.target_marker = None

        # Labels for rover and target positions
        self.position_label = tk.Label(self.root, font=("Arial", 12), bg="#f0f0f0")
        self.position_label.config(text=f"Rover position: {rover_coords.get('x', 'N/A')}, {rover_coords.get('y', 'N/A')}\n"
                                       f"Target position: {target_coords.get('x', 'N/A')}, {target_coords.get('y', 'N/A')}")
        self.position_label.pack(pady=10)

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
        self.set_target_button = tk.Button(self.root, text="Set Target", font=("Arial", 12), command=self.set_target_from_entry)
        self.set_target_button.pack(pady=10)

        # Button to center map on rover position
        self.center_rover_button = tk.Button(self.root, text="Center Map on Rover", font=("Arial", 12), command=self.center_map_on_rover)
        self.center_rover_button.pack(pady=10)

        # Add right-click menu options for the map
        self.map_widget.add_right_click_menu_command(label="Add Target", command=self.set_target, pass_coords=True)
        self.map_widget.add_right_click_menu_command(label="Move Rover", command=self.move_rover, pass_coords=True)
        self.map_widget.add_right_click_menu_command(label="Simulate", command=self.simulate_movement, pass_coords=True)

        # Start the periodic marker update
        self.update_rover_position()

        # Start ROS2 in a separate thread
        ros_thread = threading.Thread(target=self.start_ros)
        ros_thread.start()

        self.root.mainloop()

    def start_ros(self):
        rclpy.spin(self)  # Start the ROS2 spin loop in a separate thread

    def gps_callback(self, msg: NavSatFix):
        """Handle new GPS data"""
        rover_coords['x'] = msg.latitude
        rover_coords['y'] = msg.longitude
        print(rover_coords)

        # Update the position label in the main Tkinter thread
        self.root.after(0, self.update_position_label)

    def update_rover_position(self):
        global rover_coords, target_coords

        # Update the rover marker position
        self.rover_marker.set_position(rover_coords['x'], rover_coords['y'])

        # Update position label
        self.position_label.config(text=f"Rover position: {rover_coords.get('x', 'N/A')}, {rover_coords.get('y', 'N/A')}\n"
                                       f"Target position: {target_coords.get('x', 'N/A')}, {target_coords.get('y', 'N/A')}")

        # Clear the path if target exists and set new path
        self.map_widget.delete_all_path()
        if self.target_marker:
            path_1 = self.map_widget.set_path([self.rover_marker.position, self.target_marker.position])

        # Schedule the next update
        self.root.after(500, self.update_rover_position)

    def update_position_label(self):
        """Update position label with new coordinates."""
        self.position_label.config(text=f"Rover position: {rover_coords['x']}, {rover_coords['y']}\n"
                                       f"Target position: {target_coords.get('x', 'N/A')}, {target_coords.get('y', 'N/A')}")

    def simulate_movement(self, coords=None):
        global rover_coords, target_coords

        # Calculate movement vector
        dx = target_coords['x'] - rover_coords['x']
        dy = target_coords['y'] - rover_coords['y']

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

        self.root.after(500, self.simulate_movement)

    def move_rover(self, coords):
        global rover_coords
        rover_coords['x'] = coords[0]
        rover_coords['y'] = coords[1]

    def set_target(self, coords):
        global target_coords
        target_coords['x'] = coords[0]
        target_coords['y'] = coords[1]
        if self.target_marker:
            self.target_marker.delete()
        self.target_marker = self.map_widget.set_marker(target_coords['x'], target_coords['y'], text="Target")

    def set_target_from_entry(self):
        global target_coords
        try:
            lat = float(self.lat_entry.get())
            lon = float(self.lon_entry.get())
            target_coords['x'] = lat
            target_coords['y'] = lon
            self.set_target((lat, lon))
        except ValueError:
            self.show_invalid_coordinates_message()

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
    #rclpy.spin(gps_tab)
    rclpy.shutdown()
