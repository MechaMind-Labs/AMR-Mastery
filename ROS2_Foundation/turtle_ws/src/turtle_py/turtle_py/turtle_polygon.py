#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class ClosedLoopPolygonDrawer(Node):
    def __init__(self):
        super().__init__('polygon_drawer')

        # Parameters
        self.declare_parameter('sides', 3)
        self.declare_parameter('side_length', 2.0)

        self.sides = self.get_parameter('sides').value
        self.side_length = self.get_parameter('side_length').value

        # Publisher & Subscriber
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)

        # States
        self.current_pose = None
        self.start_pose = None
        self.state = "WAIT"
        self.sides_drawn = 0

        # Tolerances
        self.distance_tolerance = 0.01
        self.angle_tolerance = 0.01

        # Control gains
        self.k_linear = 1.0
        self.k_angular = 2.0

        # Speed limits
        self.max_linear_speed = 1.5
        self.max_angular_speed = 2.0

        self.get_logger().info(f"Polygon Drawer (Closed Loop) Started: sides={self.sides}, length={self.side_length}")

    def pose_callback(self, pose: Pose):
        self.current_pose = pose

    def control_loop(self):
        if self.current_pose is None:
            return

        twist = Twist()

        if self.state == "WAIT":
            self.start_pose = self.current_pose
            self.state = "FORWARD"
            self.get_logger().info("Starting first side...")

        elif self.state == "FORWARD":
            # Calculate remaining distance
            dist = math.sqrt((self.current_pose.x - self.start_pose.x) ** 2 +
                             (self.current_pose.y - self.start_pose.y) ** 2)
            error = self.side_length - dist

            if error <= self.distance_tolerance:
                twist.linear.x = 0.0
                self.cmd_pub.publish(twist)
                self.start_pose = self.current_pose
                self.state = "TURN"
                self.get_logger().info(f"Side {self.sides_drawn + 1} completed. Turning...")
            else:
                # Proportional control for linear speed
                speed = self.k_linear * error
                twist.linear.x = min(speed, self.max_linear_speed)

        elif self.state == "TURN":
            target_angle = (2 * math.pi) / self.sides
            angle_turned = self.normalize_angle(self.current_pose.theta - self.start_pose.theta)
            error = target_angle - angle_turned

            if error <= self.angle_tolerance:
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.sides_drawn += 1

                if self.sides_drawn >= self.sides:
                    self.get_logger().info("Polygon completed!")
                    rclpy.shutdown()
                    return
                else:
                    self.start_pose = self.current_pose
                    self.state = "FORWARD"
            else:
                # Proportional control for angular speed
                speed = self.k_angular * error
                twist.angular.z = min(speed, self.max_angular_speed)

        self.cmd_pub.publish(twist)

    def normalize_angle(self, angle):
        while angle < 0:
            angle += 2 * math.pi
        return angle


def main():
    rclpy.init()
    node = ClosedLoopPolygonDrawer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()