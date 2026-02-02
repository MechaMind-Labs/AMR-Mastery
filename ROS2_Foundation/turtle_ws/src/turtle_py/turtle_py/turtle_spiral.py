#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CircleDrawer(Node):
    def __init__(self):
        super().__init__("circle_mover")

        # Publisher to cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Counter
        self.counter = 0

        # Timer to publish velocity continuously
        self.timer = self.create_timer(0.1, self.move_turtle)

    def move_turtle(self):
        msg = Twist()

        # Forward speed
        msg.linear.x = 0.2 + self.counter #m/s

        # Turning speed
        msg.angular.z = 1.0 #rad/sec

        #Increment Counter
        self.counter += 0.01

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
