#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node

class RobotStatus(Node):
    def __init__(self):
        super().__init__('robot_status')
        self.time_robot_on = 0.0
    
    def robot_message(self, text, robot_name="Robot-1"):
        self.get_logger().info(robot_name+": "+text)
    
    def timer_counter(self, time_passed):
        self.time_robot_on += time_passed
        self.get_logger().info("Updated Time Robot On="+str(self.time_robot_on))

    def main_task(self):
        period = 1.0
        self.robot_message(text="Robot Booting Up...")
        time.sleep(period)
        self.timer_counter(time_passed=period)
        self.robot_message(text="Robot Ready...")
        time.sleep(period)
        self.timer_counter(time_passed=period)
        self.robot_message(text="Robot ShuttingDown...")







def main(args=None):
    rclpy.init(args=args)
    robot_status_node = RobotStatus()
    robot_status_node.main_task()
    rclpy.shutdown()

if __name__ == '__main__':
    main()