import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, CompressedImage
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class ROSNode(Node):
    def __init__(self, update_image_callback):
    #def __init__(self, update_image_callback, cmd_vel_callback):
        super().__init__('ros_node')
        self.gamepad_publisher = self.create_publisher(Joy, 'gamepad_input', 10)
        self.button_publisher = self.create_publisher(Int8MultiArray, 'button_states', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.camera_subscriptions = []
        self.update_image_callback = update_image_callback
        self.bridge = CvBridge()

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
            self.cmd_vel_publisher.publish(twist_msg)
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
        twist_msg.linear.x = self.max_linear_speed * (left_trigger + right_trigger) / 2
        twist_msg.angular.z = self.max_angular_speed * (left_trigger - right_trigger) / 2

        self.cmd_vel_publisher.publish(twist_msg)

    def publish_button_states(self, kill_switch, autonomy, manual):
        msg = Int8MultiArray()
        #msg.data = f'KillSwitch:{kill_switch};Autonomy:{autonomy};Extra:{extra}'
        msg.data = [kill_switch, autonomy, manual] #TODO: czy zmienic kolejnosc?
        self.button_publisher.publish(msg)

    def add_camera_subscription(self, topic, qos_profile):
        subscription = self.create_subscription(
            CompressedImage,  # Zmiana z Image na CompressedImage
            topic,
            lambda msg, idx=len(self.camera_subscriptions): self.camera_callback(msg, idx),
            10)
        self.camera_subscriptions.append(subscription)

    def camera_callback(self, msg, idx):
        try:
            # Dekodowanie skompresowanego obrazu
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is not None:
                self.update_image_callback(cv_image, idx)
        except Exception as e:
            self.get_logger().error(f"Błąd dekodowania obrazu: {e}")