import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import yaml
import os

class WaypointSaver(Node):
    def __init__(self):
        super().__init__('waypoint_saver')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/gps/targets',
            self.listener_callback,
            10
        )
        self.get_logger().info("WaypointSaver node started, waiting for /gps/targets...")

    def listener_callback(self, msg):
        data = msg.data
        if len(data) % 2 != 0:
            self.get_logger().warn("Received odd number of elements in GPS data array. Skipping.")
            return

        # Tworzymy listę punktów (bez dodatkowego klucza "waypoints" w każdym elemencie)
        points = []
        for i in range(0, len(data), 2):
            lat = data[i]
            lon = data[i + 1]
            point = {
                'latitude': lat,
                'longitude': lon,
                'yaw': 0.0
            }
            points.append(point)

        # Tworzymy ostateczny słownik z kluczem "waypoints" i listą punktów
        yaml_data = {'waypoints': points}

        file_path = os.path.join(os.getcwd(), 'waypoints.yaml')
        with open(file_path, 'w') as yaml_file:
            yaml.dump(yaml_data, yaml_file, default_flow_style=False)

        self.get_logger().info(f"Wrote {len(points)} waypoint entries to {file_path}")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
