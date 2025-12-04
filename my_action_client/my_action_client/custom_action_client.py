#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import math
#from leo_description.action import GoToPose
from custom_interfaces.action import GoToPose


class CustomActionServer(Node):
    def __init__(self):
        super().__init__('custom_action_client')
        self._action_client = ActionClient(self, GoToPose, 'go_to_pose')
        # Subscribe to odometry to get the robot's position
        self._odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        # Initialize positions
        self.robot_x = None
        self.robot_y = None
        self.meteor_x = None
        self.meteor_y = None
        # Subscriber to track the meteor position
        self.model_state_subscriber = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10)
        # Distance threshold to send new goal
        self.near_meteor_threshold = 7.5 # Units (e.g., meters)
        # Store goal handle to keep track of active goal
        self.goal_handle = None
        # Flag to indicate if new goal has been sent
        self.new_goal_sent = False
    
    def send_goal(self, x, y, yaw):
        goal_msg = GoToPose.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.yaw = yaw
        self.get_logger().info(f'Sending goal: x={x}, y={y}, yaw={yaw}')
        # Wait until the action server is available
        self._action_client.wait_for_server()
        # Send the goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        # Attach callback for when the goal response is received
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    
    def goal_response_callback(self, future):
        # Handle the goal response
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal was rejected by the action server.')
            return
        self.get_logger().info('Goal accepted by the action server.')
        # Attach callback for when the result is received
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    
    def get_result_callback(self, future):
        # Handle the result
        result = future.result().result
        self.get_logger().info(f'Action completed with success: {result.success}')
        # Optionally, shutdown the node or send another goal


    def feedback_callback(self, feedback_msg):
        # Handle feedback (if any)
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback from action server.')



    def odom_callback(self, msg):
        # Update robot's position
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        # Check if we have both positions and haven't sent the new goal yet
        if (self.robot_x is not None and self.robot_y is not None and
            self.meteor_x is not None and self.meteor_y is not None and
            not self.new_goal_sent):
            # Calculate distance to meteor
            distance = math.hypot(self.robot_x - self.meteor_x, self.robot_y - self.meteor_y)
            if distance <= self.near_meteor_threshold:
                self.get_logger().info('Robot is near meteor. Sending new goal to avoid it.')
                # Send new goal
                self.send_goal(-11.5, -3.5, -1.5) # next coordinates of lost astronaut
                self.new_goal_sent = True # Prevent sending multiple goals


    def model_states_callback(self, msg):
        # Extract the meteor's position from the ModelStates message
        # Get index of the meteor in the model names list
        meteor_index = msg.name.index('meteor')
        # Get the meteor's position
        self.meteor_x = msg.pose[meteor_index].position.x
        self.meteor_y = msg.pose[meteor_index].position.y


def main(args=None):
    rclpy.init(args=args)
    action_client = CustomActionServer()
    # Send the initial goal
    initial_x = -8.0
    initial_y = 6.0
    initial_yaw = 1.57
    action_client.send_goal(initial_x, initial_y, initial_yaw)
    # Keep the node alive to receive callbacks
    rclpy.spin(action_client)
    action_client.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
