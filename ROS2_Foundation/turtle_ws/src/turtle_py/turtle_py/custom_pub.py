#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import TwoInt

class NumberPublisher(Node):
    def __init__(self):
        super().__init__('two_number_publisher')
        
        # Create publisher
        self.publisher = self.create_publisher(TwoInt, 'numbers', 10)
        
        # Create timer (1 second = 1 Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Counter variable
        self.counter = 1
        
        self.get_logger().info('Number Publisher Started!')
    
    def timer_callback(self):
        # Create message
        msg = TwoInt()
        msg.a = self.counter
        msg.b = self.counter + 1
        
        # Publish message
        self.publisher.publish(msg)
        
        # Log what we published
        self.get_logger().info(f'Publishing: {self.counter} & {self.counter + 1}')
        
        # Increment counter
        self.counter += 1

def main():
    rclpy.init()
    node = NumberPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()