#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')
        
        # Create publisher
        self.publisher = self.create_publisher(Int32, 'numbers', 10)
        
        # Create timer (1 second = 1 Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Counter variable
        self.counter = 1
        
        self.get_logger().info('Number Publisher Started!')
    
    def timer_callback(self):
        # Create message
        msg = Int32()
        msg.data = self.counter
        
        # Publish message
        self.publisher.publish(msg)
        
        # Log what we published
        self.get_logger().info(f'Publishing: {self.counter}')
        
        # Increment counter
        self.counter += 1

def main():
    rclpy.init()
    node = NumberPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()