#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.srv import ComputeArea

class AreaServiceServer(Node):
    def __init__(self):
        super().__init__('area_service_server')
        self.srv = self.create_service(ComputeArea, 'calculate_area', self.calculate_area_callback)
        self.get_logger().info("Area Service Server is ready...")

    def calculate_area_callback(self, request, response):
        self.get_logger().info(f"Received request: length={request.length}, width={request.width}")
        response.area = request.length * request.width
        self.get_logger().info(f"Sending response: area={response.area}")
        return response

def main():
    rclpy.init()
    node = AreaServiceServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()