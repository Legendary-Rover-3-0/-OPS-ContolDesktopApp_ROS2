import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, CompressedImage
from std_msgs.msg import Int8MultiArray, String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import serial


class ROSNode(Node):
    def __init__(self):
    #def __init__(self, update_image_callback):
        super().__init__('ros_node')
        self.gamepad_publisher = self.create_publisher(Joy, 'gamepad_input', 10)
        self.button_publisher = self.create_publisher(Int8MultiArray, '/ESP32_GIZ/led_state_topic', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel_nav', 10) 

        self.communication_mode = 'ROS2' #ROS2 lub SATEL
        self.serial_port = None
        
        # self.camera_subscriptions = []
        # self.update_image_callback = update_image_callback
        self.bridge = CvBridge()
        
        self.speed_factor = 1.0  

        # Skalowanie prędkości
        self.max_linear_speed = 1.0  # Maksymalna prędkość liniowa (m/s)
        self.max_angular_speed = 1.0  # Maksymalna prędkość kątowa (rad/s)

        # # Subskrypcja /cmd_vel
        # self.cmd_vel_subscription = self.create_subscription(
        #     Twist, '/cmd_vel', self.cmd_vel_listener, 10
        # )

    # def cmd_vel_listener(self, msg):
    #     # Przekazanie danych z /cmd_vel do callbacka
    #     if self.cmd_vel_callback:
    #         self.cmd_vel_callback(msg)

    def publish_gamepad_input(self, buttons, axes, hat=(0, 0)):  
        hat_x, hat_y = float(hat[0]), float(hat[1])  # Konwersja int -> float
        axes.extend([hat_x, hat_y])  # Dodanie poprawnych wartości do listy axes

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = axes  # Teraz zawiera także hat_x i hat_y jako float
        msg.buttons = buttons  
        self.gamepad_publisher.publish(msg)

        self.publish_cmd_vel(axes, buttons)  

    
    def publish_empty_gamepad_input(self):  
        msg = Joy()
        self.gamepad_publisher.publish(msg)

        self.publish_cmd_vel()  


    def publish_cmd_vel(self, axes=None, buttons=None):
        if axes is None or buttons is None:
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            if self.communication_mode == 'ROS2':
                self.cmd_vel_publisher.publish(twist_msg)
            elif self.communication_mode == 'SATEL':
                self.send_serial_frame("DV", 128, 128)
            return

        if len(axes) < 6 or len(buttons) < 6:
            self.get_logger().error("Za mało danych z gamepada!")
            return

        reverse_mode_left = buttons[5]
        reverse_mode_right = buttons[4]

        left_trigger = (axes[5] + 1) / 2 * (-1 if reverse_mode_left else 1)
        right_trigger = (axes[2] + 1) / 2 * (-1 if reverse_mode_right else 1)

        max_trigger = max(left_trigger, right_trigger)

        twist_msg = Twist()
        twist_msg.linear.x = self.max_linear_speed * (left_trigger + right_trigger) / 2 * self.speed_factor
        twist_msg.angular.z = self.max_angular_speed * (left_trigger - right_trigger) / 2 * self.speed_factor

        # Zakoduj x i z do bajtów
        x_byte = self.float_to_byte(twist_msg.linear.x)
        z_byte = self.float_to_byte(twist_msg.angular.z)

        # Wyślij ramkę
        if self.communication_mode == 'ROS2':
            self.cmd_vel_publisher.publish(twist_msg)
        elif self.communication_mode == 'SATEL':
            self.send_serial_frame("DV", x_byte, z_byte)


    def publish_button_states(self, kill_switch, autonomy, manual):
        if self.communication_mode == 'ROS2':
            msg = Int8MultiArray()
            msg.data = [manual, autonomy, kill_switch]
            self.button_publisher.publish(msg)
        elif self.communication_mode == 'SATEL':
            byte = (manual << 0) | (autonomy << 1) | (kill_switch << 2)
            self.send_serial_frame("GL", byte)

    def update_speed_factor(self, factor):
        self.speed_factor = factor  

    def float_to_byte(self, value):
        """Mapowanie float z [-1, 1] na bajt [0, 254]."""
        value = max(-1.0, min(1.0, value))  # clamp
        #return round((value + 1.0) / 2.0 * 255)
        return int((value*127.0)+128)

    def float_to_byte_100(self, value):
        """Mapowanie float z [-100, 100] na bajt [0, 254]."""
        value = max(-100.0, min(100.0, value))  # clamp
        return int((value + 100.0) / 200.0 * 254)

    def send_serial_frame(self, mark, *bytes):
        try:
            # Prosty checksum: suma x + z modulo 256
            sum = checksum = 0
            for byte in bytes:
                sum += byte
            checksum = sum % 256

            frame = bytearray()
            frame.extend(b"$")
            frame.extend(mark.encode('utf-8'))
            for byte in bytes:
                frame.append(byte)
            frame.append(checksum)
            frame.extend(b"#")
            # print(frame)

            bit_string = ' '.join(f'{byte:08b}' for byte in frame)
            # print(bit_string)

            self.serial_port.write(frame)

        except Exception as e:
            print(f"Błąd przy wysyłaniu ramki szeregowej: {e}")
    # def add_camera_subscription(self, topic, qos_profile):
    #     subscription = self.create_subscription(
    #         CompressedImage,  # Zmiana z Image na CompressedImage
    #         topic,
    #         lambda msg, idx=len(self.camera_subscriptions): self.camera_callback(msg, idx),
    #         10)
    #     self.camera_subscriptions.append(subscription)

    # def camera_callback(self, msg, idx):
    #     try:
    #         # Dekodowanie skompresowanego obrazu
    #         np_arr = np.frombuffer(msg.data, np.uint8)
    #         cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    #         if cv_image is not None:
    #             self.update_image_callback(cv_image, idx)
    #     except Exception as e:
    #         self.get_logger().error(f"Błąd dekodowania obrazu: {e}")