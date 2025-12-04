#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from leo_description.action import Rotate
from rclpy.action import ActionServer

from geometry_msgs.msg import Twist
import time

class RotateActionServer(Node):

    def __init__(self):
        super().__init__('rotate_action_server')

        # Create an action server
        self._action_server = ActionServer(
            self,
            Rotate,
            'rotate',
            self.execute_callback)

        # Publisher to send velocity commands
        self._cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        rotation_time = goal_handle.request.rotation_time
        angular_speed = 0.6  # radians per second

        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = angular_speed

        start_time = self.get_clock().now().nanoseconds / 1e9  # Convert nanoseconds to seconds

        feedback_msg = Rotate.Feedback()

        while rclpy.ok():
            # Check for cancellation request
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                twist_msg.angular.z = 0.0
                self._cmd_vel_publisher.publish(twist_msg)
                goal_handle.canceled()
                return Rotate.Result(success=False)

            current_time = self.get_clock().now().nanoseconds / 1e9
            elapsed_time = current_time - start_time

            # Publish feedback
            feedback_msg.elapsed_time = elapsed_time
            goal_handle.publish_feedback(feedback_msg)

            # Check if the rotation time has passed
            if elapsed_time >= rotation_time:
                self.get_logger().info('Rotation time reached')
                break

            # Publish the rotation command
            self._cmd_vel_publisher.publish(twist_msg)

            # Sleep for a short duration
            time.sleep(0.1)

        # Stop the robot
        twist_msg.angular.z = 0.0
        self._cmd_vel_publisher.publish(twist_msg)

        goal_handle.succeed()
        result = Rotate.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = RotateActionServer()
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
