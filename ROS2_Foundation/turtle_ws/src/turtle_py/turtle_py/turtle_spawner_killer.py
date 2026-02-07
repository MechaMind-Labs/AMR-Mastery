#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
import random

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        
        self.turtle_names = []
        self.counter = 0
        
        # Create service clients
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.kill_client = self.create_client(Kill, '/kill')
        
        # Wait for services
        self.spawn_client.wait_for_service()
        self.kill_client.wait_for_service()
        
        # Create timers
        self.spawn_timer = self.create_timer(1.0, self.spawn_turtle)
        self.kill_timer = self.create_timer(2.0, self.kill_turtle)
        
        self.get_logger().info("Turtle spawner started")
    
    def spawn_turtle(self):
        self.counter += 1
        name = f"turtle{self.counter}"
        
        request = Spawn.Request()
        request.x = random.uniform(1.0, 10.0)
        request.y = random.uniform(1.0, 10.0)
        request.theta = random.uniform(0.0, 6.28)
        request.name = name
        
        future = self.spawn_client.call_async(request)
        future.add_done_callback(lambda f: self.spawn_callback(f, name))
    
    def spawn_callback(self, future, name):
        try:
            future.result()
            self.turtle_names.append(name)
            self.get_logger().info(f"Spawned {name}")
        except Exception as e:
            self.get_logger().error(f"Spawn failed: {e}")
    
    def kill_turtle(self):
        if self.turtle_names:
            name = self.turtle_names.pop(0)
            
            request = Kill.Request()
            request.name = name
            
            future = self.kill_client.call_async(request)
            future.add_done_callback(lambda f: self.kill_callback(f, name))
    
    def kill_callback(self, future, name):
        try:
            future.result()
            self.get_logger().info(f"Killed {name}")
        except Exception as e:
            self.get_logger().error(f"Kill failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()