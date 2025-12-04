#! /usr/bin/env python
import rclpy
import time
from rclpy.node import Node

class HeartbeatNode(Node):
    def __init__(self, rover_name, timer_period=0.2):
        self._rover_name = rover_name
        super().__init__(self._rover_name)
        self.create_timer(timer_period, self.timer_callback)
    def timer_callback(self):
        ros_time_stamp = self.get_clock().now()
        self.get_logger().info(self._rover_name + " is alive ... " + str(ros_time_stamp))

def main1(args=None):
    rclpy.init(args=args)
    node = HeartbeatNode(rover_name='leo_rover_1', timer_period=1.0)
    rclpy.spin(node)
    rclpy.shutdown()

def main2(args=None):
    rclpy.init(args=args)
    node = HeartbeatNode(rover_name='leo_rover_2', timer_period=1.0)
    rclpy.spin(node)
    rclpy.shutdown()


def main_shutdown(args=None):
    rclpy.init(args=args)
    print('Shutting down leo rover 1...')
    rclpy.shutdown()


if __name__ == '__main__':
    main1()