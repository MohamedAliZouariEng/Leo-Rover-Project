#! /usr/bin/env python

import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist 
import signal

class MoveRoverNode(Node):
    def __init__(self):
        super().__init__('leo_rover_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -0.5
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing : "%s"'%msg)
    
    def stop_rover(self):
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        self.get_logger().info('Publishing stop message before shutdown')


def main(args=None):
    rclpy.init(args=args)
    simple_publisher = MoveRoverNode()

    def signal_handler(sig, frame):
        simple_publisher.stop_rover()
        simple_publisher.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)
    rclpy.spin(simple_publisher)

if __name__ == '__main__':
    main()