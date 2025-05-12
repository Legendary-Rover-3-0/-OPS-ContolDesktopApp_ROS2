import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, CompressedImage
from std_msgs.msg import Int8MultiArray, String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading # Import threading

class ROSNode(Node):
    def __init__(self):
        super().__init__('ros_node')
        self.gamepad_publisher = self.create_publisher(Joy, 'gamepad_input', 10)
        self.button_publisher = self.create_publisher(Int8MultiArray, '/ESP32_GIZ/led_state_topic', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        self.bridge = CvBridge()

        self.speed_factor = 1.0

        # Skalowanie prędkości
        self.max_linear_speed = 1.0  # Maksymalna prędkość liniowa (m/s)
        self.max_angular_speed = 1.0  # Maksymalna prędkość kątowa (rad/s)

        self._timed_move_active = False # Flag to indicate if a timed move is active
        self._timed_move_timer = None # Timer for timed moves

    def publish_gamepad_input(self, buttons, axes, hat=(0, 0)):
        if self._timed_move_active:
            return # Do not publish gamepad input if timed move is active

        hat_x, hat_y = float(hat[0]), float(hat[1])
        axes.extend([hat_x, hat_y])

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = axes
        msg.buttons = buttons
        self.gamepad_publisher.publish(msg)

        self.publish_cmd_vel(axes, buttons)


    def publish_empty_gamepad_input(self):
        if self._timed_move_active:
            return # Do not publish empty input if timed move is active
            
        msg = Joy()
        self.gamepad_publisher.publish(msg)

        self.publish_cmd_vel()


    def publish_cmd_vel(self, axes=None, buttons=None):
        if self._timed_move_active:
            return # Do not publish gamepad cmd_vel if timed move is active

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

        twist_msg = Twist()
        twist_msg.linear.x = self.max_linear_speed * (left_trigger + right_trigger) / 2 * self.speed_factor
        twist_msg.angular.z = self.max_angular_speed * (left_trigger - right_trigger) / 2 * self.speed_factor

        self.cmd_vel_publisher.publish(twist_msg)

    def publish_button_states(self, kill_switch, autonomy, manual):
        msg = Int8MultiArray()
        msg.data = [manual, autonomy, kill_switch]
        self.button_publisher.publish(msg)

    def update_speed_factor(self, factor):
        self.speed_factor = factor

    def publish_cmd_vel_timed(self, linear_x, angular_z):
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist_msg)