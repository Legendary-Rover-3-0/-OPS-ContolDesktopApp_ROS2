import os
import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32

# Upewnij się, że folder 'Science_data_backup' istnieje
backup_directory = "Science_data_backup"
os.makedirs(backup_directory, exist_ok=True)

class ScienceBackupNode(Node):
    def __init__(self):
        super().__init__('science_backup_node')

        # Subskrypcje danych
        self.create_subscription(Float32MultiArray, 'CO2_publisher', self.co2_callback, 10)
        self.create_subscription(Float32MultiArray, 'Methane_publisher', self.methane_callback, 10)
        self.create_subscription(Float32, 'Radiation_Publisher', self.radiation_callback, 10)
        self.create_subscription(Float32, 'Temp_Publisher', self.temp_callback, 10)
        self.create_subscription(Float32, 'Humidity_Publisher', self.humidity_callback, 10)

    def co2_callback(self, msg):
        """Callback do zapisywania danych CO2"""
        if len(msg.data) >= 2:
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            with open(f"{backup_directory}/co2.txt", "a") as f:
                f.write(f"{timestamp}, {msg.data[0]:.1f}, {msg.data[1]:.1f}\n")

    def methane_callback(self, msg):
        """Callback do zapisywania danych Metanu"""
        if len(msg.data) >= 2:
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            with open(f"{backup_directory}/methane.txt", "a") as f:
                f.write(f"{timestamp}, {msg.data[0]:.2f}, {msg.data[1]:.2f}\n")

    def radiation_callback(self, msg):
        """Callback do zapisywania danych Radiacji"""
        radiation_value = msg.data
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(f"{backup_directory}/radiation.txt", "a") as f:
            f.write(f"{timestamp}, {radiation_value:.1f}\n")

    def temp_callback(self, msg):
        """Callback do zapisywania danych temperatury"""
        temp_value = msg.data
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(f"{backup_directory}/soil_temp.txt", "a") as f:
            f.write(f"{timestamp}, {temp_value:.1f}\n")

    def humidity_callback(self, msg):
        """Callback do zapisywania danych Wilgotności"""
        humidity_value = msg.data
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(f"{backup_directory}/soil_humidity.txt", "a") as f:
            f.write(f"{timestamp}, {humidity_value:.1f}\n")

def main():
    """Inicjalizowanie ROS 2 i uruchomienie węzła"""
    rclpy.init()
    
    node = ScienceBackupNode()

    # Utrzymywanie procesu
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
