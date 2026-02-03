#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleCirclePublisher(Node):
    def __init__(self):
        super().__init__('turtle_circle_publisher')
        
        # Create publisher to cmd_vel topic
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Create timer (10 Hz = 0.1 seconds)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Turtle Circle Publisher Started!')
        self.get_logger().info('Publishing commands to draw a circle...')
    
    def timer_callback(self):
        # Create Twist message
        msg = Twist()
        
        # Set linear and angular velocity to draw circle
        msg.linear.x = 2.0      # Forward speed
        msg.angular.z = 1.0     # Turning speed (creates circular motion)
        
        # Publish message
        self.publisher.publish(msg)
        
        # Log what we're publishing
        self.get_logger().info(f'Publishing -> Linear: {msg.linear.x}, Angular: {msg.angular.z}')

def main():
    rclpy.init()
    node = TurtleCirclePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()