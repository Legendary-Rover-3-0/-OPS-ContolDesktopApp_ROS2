import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import serial.tools.list_ports
import threading
from std_msgs.msg import Int8MultiArray, Int32MultiArray, Int16, Float64MultiArray
import argparse

debug = True
prevent_ROS = False

class SerialReceiverNode(Node):
    def __init__(self, port_name: str):
        super().__init__('serial_receiver')
        self.drive_publisher = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.mani_publisher = self.create_publisher(Float64MultiArray, '/array_topic', 10)
        self.button_publisher = self.create_publisher(Int8MultiArray, '/ESP32_GIZ/led_state_topic', 10)
        self.servo_publisher = self.create_publisher(Int32MultiArray, '/ESP32_GIZ/servo_angles_topic', 10)
        self.koszelnik_publisher = self.create_publisher(Int8MultiArray, '/ESP32_GIZ/output_state_topic', 10)
        self.science_servo_publisher = self.create_publisher(Int32MultiArray, '/servos_urc_control', 10)
        self.science_pump_publisher = self.create_publisher(Int8MultiArray, '/pumps_urc_control', 10)
        self.science_led_publisher = self.create_publisher(Int16, '/led_urc_control', 10)

        try:

            self.serial_port = serial.Serial(port_name, 9600, timeout=0.1)
        except Exception as e:
            self.get_logger().error(f"Nie udało się otworzyć portu szeregowego: {e}")
            self.serial_port = None

        if self.serial_port:
            self.serial_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
            self.serial_thread.start()

    def serial_read_loop(self):
        buffer = bytearray()

        while rclpy.ok():
            try:
                byte = self.serial_port.read(1)
                if not byte:
                    continue

                buffer.extend(byte)

                while True:
                    start_idx = buffer.find(b"$")
                    end_idx = buffer.find(b"#", start_idx + 1)

                    if start_idx == -1 or end_idx == -1:
                        break  # brak pełnej ramki

                    frame = buffer[start_idx + 1:end_idx]  # bez $ i #
                    del buffer[:end_idx + 1]  # usuń zużytą ramkę

                    self.parse_frame(frame)

            except Exception as e:
                self.get_logger().error(f"Błąd odczytu z UART: {e}")

    def parse_frame(self, frame: bytearray):
        try:
            if len(frame) < 3:
                if debug: self.get_logger().warn("Ramka za krótka.")
                return

            # Rozpoznaj typ ramki po pierwszych bajtach
            header = frame[0:2]

            if header == b'DV' and len(frame) == 5:
                x_byte = frame[2]
                z_byte = frame[3]
                checksum = frame[4]

                if (x_byte + z_byte) % 256 != checksum:
                    if debug: self.get_logger().warn("Nieprawidłowa suma kontrolna.")
                    return

                x = self.byte_to_float(x_byte)
                z = self.byte_to_float(z_byte)

                twist = Twist()
                twist.linear.x = x
                twist.angular.z = z
                if not prevent_ROS: self.drive_publisher.publish(twist)

                if debug: self.get_logger().info(f"Odebrano DV: x={x:.2f}, z={z:.2f}")

            elif header == b'MN' and len(frame) == 9:
                val1 = frame[2]
                val2 = frame[3]
                val3 = frame[4]
                val4 = frame[5]
                val5 = frame[6]
                val6 = frame[7]
                checksum = frame[8]

                if (val1 + val2 + val3 + val4 + val5 + val6) % 256 != checksum:
                    if debug: self.get_logger().warn("Nieprawidłowa suma kontrolna.")
                    return
                
                val1 = self.byte_to_float_100(val1)
                val2 = self.byte_to_float_100(val2)
                val3 = self.byte_to_float_100(val3)
                val4 = self.byte_to_float_100(val4)
                val5 = self.byte_to_float_100(val5)
                val6 = self.byte_to_float_100(val6)
                
                msg = Float64MultiArray()
                msg.data = [0.0] * 6 
                msg.data[0] = val1
                msg.data[1] = val2
                msg.data[2] = val3
                msg.data[3] = val4
                msg.data[4] = val5
                msg.data[5] = val6

                if not prevent_ROS: self.mani_publisher.publish(msg)
                if debug: self.get_logger().info(f"Odebrano MN: val1={val1:.2f}, val2={val2:.2f}, val3={val3},\
val4={val4:.2f}, val5={val5:.2f}, val6={val6:.2f}")
                
            elif header == b'GL' and len(frame) == 4:
                byte = frame[2]
                manual = (byte >> 0) & 1
                autonomy = (byte >> 1) & 1
                kill_switch = (byte >> 2) & 1
                checksum = frame[3]

                if byte % 256 != checksum:
                    if debug: self.get_logger().warn("Nieprawidłowa suma kontrolna.")
                    return

                msg = Int8MultiArray()
                msg.data = [manual, autonomy, kill_switch]
                if not prevent_ROS: self.button_publisher.publish(msg)
                if debug: self.get_logger().info(f"Odebrano GL: manual={manual}, autonomy={autonomy}, kill_switch={kill_switch}")

            elif header == b'GS' and len(frame) == 7:
                s1 = frame[2]
                s2 = frame[3]
                s3 = frame[4]
                s4 = frame[5]
                checksum = frame[6]

                if (s1+s2+s3+s4) % 256 != checksum:
                    if debug: self.get_logger().warn("Nieprawidłowa suma kontrolna.")
                    return

                msg = Int32MultiArray()
                msg.data = [s1, s2, s3, s4]
                if not prevent_ROS: self.servo_publisher.publish(msg)
                if debug: self.get_logger().info(f"Odebrano GS: servo_1={s1}, servo_2={s2}, servo_3={s3}, servo_4={s4}")

            elif header == b'GK' and len(frame) == 4:
                byte = frame[2]
                drill = (byte >> 0) & 1
                koszelnik = (byte >> 1) & 1
                heater = (byte >> 2) & 1
                checksum = frame[3]

                if byte % 256 != checksum:
                    if debug: self.get_logger().warn("Nieprawidłowa suma kontrolna.")
                    return

                msg = Int8MultiArray()
                msg.data = [drill, koszelnik, heater]
                if not prevent_ROS: self.koszelnik_publisher.publish(msg)
                if debug: self.get_logger().info(f"Odebrano GK: drill={drill}, koszelnik={koszelnik}, heater={heater}")

            elif header == b'SS' and len(frame) == 9:
                s_vals = list(frame[2:8])  # 6 serw
                checksum = frame[8]

                if sum(s_vals) % 256 != checksum:
                    if debug: self.get_logger().warn("Nieprawidłowa suma kontrolna (SS).")
                    return

                msg = Int32MultiArray()
                msg.data = s_vals
                if not prevent_ROS: self.science_servo_publisher.publish(msg)
                if debug: self.get_logger().info(f"Odebrano SS: s1={s_vals[0]}, s2={s_vals[1]}, s3={s_vals[2]}, s4={s_vals[3]}, s5={s_vals[4]}, s6={s_vals[5]}")
            
            elif header == b'SL' and len(frame) == 4:
                brightness = frame[2]
                checksum = frame[3]

                if brightness % 256 != checksum:
                    if debug: self.get_logger().warn("Nieprawidłowa suma kontrolna (SL).")
                    return

                msg = Int16()
                msg.data = brightness
                if not prevent_ROS: self.science_led_publisher.publish(msg)
                if debug: self.get_logger().info(f"Odebrano SL: LED={brightness}")

            elif header == b'SP' and len(frame) == 4:
                pumps_byte = frame[2]
                checksum = frame[3]

                if pumps_byte % 256 != checksum:
                    if debug: self.get_logger().warn("Nieprawidłowa suma kontrolna (SP).")
                    return

                # Rozbicie na 2 pompki (bit 0 i 1)
                p1 = (pumps_byte >> 0) & 0x01
                p2 = (pumps_byte >> 1) & 0x01

                msg = Int8MultiArray()
                msg.data = [p1, p2]
                if not prevent_ROS: self.science_pump_publisher.publish(msg)
                if debug: self.get_logger().info(f"Odebrano SP: pompa1={p1}, pompa2={p2}")

            else:
                if debug: self.get_logger().warn(f"Nieznana lub nieobsługiwana ramka: {frame}")

        except Exception as e:
            if debug: self.get_logger().error(f"Błąd parsowania ramki: {e}")

    def byte_to_float(self, b):
        """Mapowanie bajtu [0, 254] z powrotem na float [-1, 1]."""
        return (b - 128) / 127.0
    
    def byte_to_float_100(self, byte_value):
        """Mapowanie bajtu [0, 254] z powrotem na float [-100, 100]."""
        byte_value = max(0, min(254, byte_value))  # clamp
        return (byte_value / 254.0) * 200.0 - 100.0


def main():
    parser = argparse.ArgumentParser(description='Serial receiver node')
    parser.add_argument('port', type=str, help='Ścieżka do portu szeregowego (np. /dev/ttyUSB0)')
    args = parser.parse_args()

    rclpy.init()
    node = SerialReceiverNode(port_name=args.port)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_port:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
