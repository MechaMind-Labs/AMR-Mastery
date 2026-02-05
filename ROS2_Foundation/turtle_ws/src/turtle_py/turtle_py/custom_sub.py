#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import TwoInt

class NumberSubscriber(Node):
    def __init__(self):
        super().__init__('two_number_subscriber')
        
        # Create subscriber
        self.subscription = self.create_subscription(TwoInt, 'numbers', self.listener_callback, 10)
        
        # Sum variable
        self.sum = 0
        
        self.get_logger().info('Number Subscriber Started!')
    
    def listener_callback(self, msg):
        # Add received number to sum
        self.sum = msg.a + msg.b
        
        # Log received number and current sum
        self.get_logger().info(f'Received: {msg.a} & {msg.b} Sum: {self.sum}')

def main():
    rclpy.init()
    node = NumberSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()