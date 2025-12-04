#! /usr/bin/env python
import rclpy
import random
from rclpy.node import Node

class TemperatureMonitorNode(Node):


    def __init__(self):        
        super().__init__("temperature_monitor_node")
        self.create_timer(1.0, self.monitor_callback)
    

    def monitor_callback(self):
        temperature = random.uniform(20.0, 100.0)
        if temperature > 70.0:
            self.get_logger().warn("Warning: High temperature detected!" + str(temperature) + "°C")
        else:
            self.get_logger().info("Current temperature: " + str(temperature) + "°C")




def start_monitor(args=None):
    rclpy.init(args=args)
    node = TemperatureMonitorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    start_monitor()