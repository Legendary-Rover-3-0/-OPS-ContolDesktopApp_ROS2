import os
import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import NavSatFix
from geopy.distance import geodesic
import sqlite3

# Upewnij się, że folder 'Science_data_backup' istnieje
backup_directory = "Science_data_backup"
os.makedirs(backup_directory, exist_ok=True)

class ScienceBackupNode(Node):
    def __init__(self):
        super().__init__('science_backup_node')
        
        self.latest_lat = None
        self.latest_lon = None

        # Subskrypcje danych
        self.create_subscription(Float32MultiArray, 'CO2_publisher', self.co2_callback, 10)
        self.create_subscription(Float32MultiArray, 'Methane_publisher', self.methane_callback, 10)
        self.create_subscription(Float32, 'Radiation_Publisher', self.radiation_callback, 10)
        self.create_subscription(Float32, 'Temp_Publisher', self.temp_callback, 10)
        self.create_subscription(Float32, 'Humidity_Publisher', self.humidity_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)

        self.init_radiation_database()

    def init_radiation_database(self):
        os.makedirs("GPS", exist_ok=True)
        conn = sqlite3.connect("GPS/radiation_data.db")
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

    def gps_callback(self, msg):
            self.latest_lat = msg.latitude
            self.latest_lon = msg.longitude

    def radiation_callback(self, msg):
        radiation_value = msg.data
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Zapis do pliku tekstowego (jak wcześniej)
        with open(f"{backup_directory}/radiation.txt", "a") as f:
            f.write(f"{timestamp}, {radiation_value:.1f}\n")

        # Zapis do bazy danych z GPS
        if self.latest_lat is None or self.latest_lon is None:
            self.get_logger().warn("Brak danych GPS - pomiar nie zostanie zapisany do bazy.")
            return

        try:
            conn = sqlite3.connect("GPS/radiation_data.db")
            cursor = conn.cursor()

            # Sprawdź czy istnieje punkt w promieniu <10m
            cursor.execute('SELECT latitude, longitude FROM radiation')
            for lat, lon in cursor.fetchall():
                if geodesic((lat, lon), (self.latest_lat, self.latest_lon)).meters < 10:
                    # Aktualizuj istniejący wpis
                    cursor.execute('''
                        UPDATE radiation
                        SET radiation = ?, timestamp = ?
                        WHERE latitude = ? AND longitude = ?
                    ''', (radiation_value, timestamp, lat, lon))
                    conn.commit()
                    conn.close()
                    return

            # Dodaj nowy wpis
            cursor.execute('''
                INSERT INTO radiation (latitude, longitude, radiation, timestamp)
                VALUES (?, ?, ?, ?)
            ''', (self.latest_lat, self.latest_lon, radiation_value, timestamp))
            conn.commit()
            conn.close()

        except sqlite3.Error as e:
            self.get_logger().error(f"Błąd zapisu do bazy danych radiacji: {e}")

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
