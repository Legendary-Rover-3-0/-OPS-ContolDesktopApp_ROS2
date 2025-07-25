#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
import time

class LEDController(Node):
    def __init__(self):
        super().__init__('led_controller')
        self.publisher = self.create_publisher(Int8MultiArray, '/ESP32_GIZ/led_state_topic', 10)
        time.sleep(1)
        
    def set_leds(self, led2, led1, led3):
        """Ustawia LED-y w kolejności: górny (2), środkowy (1), dolny (3)"""
        msg = Int8MultiArray()
        msg.data = [led2, led1, led3]  # Kolejność zgodna z Twoim oryginalnym kodem
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        time.sleep(0.25)  # Optymalny czas dla dobrego efektu wizualnego

def main(args=None):
    rclpy.init(args=args)
    led_controller = LEDController()
    
    try:
        while True:
            # Efekt "spływania w dół" - 3 cykle
            for _ in range(3):
                led_controller.set_leds(1, 0, 0)  # Tylko górny (2)
                led_controller.set_leds(0, 1, 0)  # Tylko środkowy (1)
                led_controller.set_leds(0, 0, 1)  # Tylko dolny (3)
                led_controller.set_leds(0, 0, 0)  # Wszystkie wyłączone
            
            # Efekt "przepływu" w dół - 3 cykle
            for _ in range(3):
                led_controller.set_leds(1, 0, 0)  # Tylko górny
                led_controller.set_leds(1, 1, 0)  # Górny + środkowy
                led_controller.set_leds(0, 1, 0)  # Tylko środkowy
                led_controller.set_leds(0, 1, 1)  # Środkowy + dolny
                led_controller.set_leds(0, 0, 1)  # Tylko dolny
                led_controller.set_leds(0, 0, 0)  # Wyłącz wszystkie
            
            # Szybkie miganie wszystkimi - 5 cykli
            for _ in range(5):
                led_controller.set_leds(1, 1, 1)
                time.sleep(0.1)
                led_controller.set_leds(0, 0, 0)
                time.sleep(0.1)
            
            # Efekt "spływania w górę" - 3 cykle
            for _ in range(3):
                led_controller.set_leds(0, 0, 1)  # Tylko dolny (3)
                led_controller.set_leds(0, 1, 0)  # Tylko środkowy (1)
                led_controller.set_leds(1, 0, 0)  # Tylko górny (2)
                led_controller.set_leds(0, 0, 0)  # Wszystkie wyłączone
            
            # Efekt "przepływu" w górę - 3 cykle
            for _ in range(3):
                led_controller.set_leds(0, 0, 1)  # Tylko dolny
                led_controller.set_leds(0, 1, 1)  # Dolny + środkowy
                led_controller.set_leds(0, 1, 0)  # Tylko środkowy
                led_controller.set_leds(1, 1, 0)  # Środkowy + górny
                led_controller.set_leds(1, 0, 0)  # Tylko górny
                led_controller.set_leds(0, 0, 0)  # Wyłącz wszystkie
            
            # "Biegająca kropka" - 5 cykli
            for _ in range(5):
                led_controller.set_leds(1, 0, 0)
                led_controller.set_leds(0, 1, 0)
                led_controller.set_leds(0, 0, 1)
                led_controller.set_leds(0, 1, 0)
            
            # "Eksplozja" - rozchodzenie się od środka
            for _ in range(3):
                led_controller.set_leds(0, 1, 0)  # Tylko środkowy
                led_controller.set_leds(1, 0, 1)  # Górny + dolny
                led_controller.set_leds(0, 1, 0)  # Tylko środkowy
                led_controller.set_leds(1, 1, 1)  # Wszystkie
                led_controller.set_leds(0, 0, 0)  # Wyłącz
            
    except KeyboardInterrupt:
        led_controller.get_logger().info('Zamykanie LED controller...')
        led_controller.set_leds(0, 0, 0)
        led_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
