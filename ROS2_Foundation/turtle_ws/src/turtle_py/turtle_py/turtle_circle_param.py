#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CircleDrawer(Node):
    def __init__(self):
        super().__init__("circle_mover")

        # Declare parameters with default values
        self.declare_parameter("linear_speed", 2.0)
        self.declare_parameter("angular_speed", 1.0)

        # Get parameter values at launch
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value

        # Publisher to cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Timer to publish velocity continuously
        self.timer = self.create_timer(0.1, self.move_turtle)

    def move_turtle(self):
        msg = Twist()

        # Get parameter values in real-time
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value

        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
