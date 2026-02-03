#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtlePoseSubscriber(Node):
    def __init__(self):
        super().__init__('turtle_pose_subscriber')
        
        # Create subscriber to pose topic
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        self.get_logger().info('Turtle Pose Subscriber Started!')
        self.get_logger().info('Listening to turtle pose...')
    
    def pose_callback(self, msg):
        # Print all pose information
        self.get_logger().info(f'Position -> x: {msg.x:.2f}, y: {msg.y:.2f}')
        self.get_logger().info(f'Orientation -> theta: {msg.theta:.2f}')
        self.get_logger().info(f'Linear Velocity: {msg.linear_velocity:.2f}')
        self.get_logger().info(f'Angular Velocity: {msg.angular_velocity:.2f}')
        self.get_logger().info('---')

def main():
    rclpy.init()
    node = TurtlePoseSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()