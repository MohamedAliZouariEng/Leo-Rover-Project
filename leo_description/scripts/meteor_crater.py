#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import math

class MeteorCraterNode(Node):
    def __init__(self):
        super().__init__('meteor_crater_node')
        self._client = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        # Wait for the SetEntityState service to be available
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state service...')

        self._model_state_subscriber = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )
        self._odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.meteor_landed = False
        self.meteor_name = 'meteor'
        self.x_crash = -9
        self.y_crash = 6
        self.meteor_height = 10

        # Variables to store robot position
        self.robot_x = None
        self.robot_y = None
        self.robot_threshold = 2.8  # Distance threshold

        # Flag to check if the meteor has been shot
        self.meteor_shot = False

    def odom_callback(self, msg):
        # Get the robot's current position from the /odom topic
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Calculate the distance to the crash point
        distance_to_crash = math.sqrt(
            (self.robot_x - self.x_crash) ** 2 + (self.robot_y - self.y_crash) ** 2
        )

        if distance_to_crash <= self.robot_threshold and not self.meteor_shot:
            self.get_logger().info('Robot has reached the target position. Shooting meteor...')
            self.set_entity_state('meteor', x=self.x_crash, y=self.y_crash, z=self.meteor_height)
            self.meteor_shot = True

    def model_states_callback(self, msg):
        # Check if the meteor has landed
        if self.meteor_name in msg.name:
            meteor_index = msg.name.index(self.meteor_name)
            meteor_pose = msg.pose[meteor_index]
            meteor_z_position = meteor_pose.position.z

            if meteor_z_position <= 0.8 and not self.meteor_landed and self.meteor_shot:
                self.meteor_landed = True
                self.get_logger().info('Meteor has landed! Spawning crater...')
                # Spawning the crater after the meteor lands
                self.set_entity_state('crater', x=self.x_crash, y=self.y_crash, z=-0.05)

    def set_entity_state(self, name, x, y, z):
        request = SetEntityState.Request()
        request.state.name = name
        request.state.pose.position.x = float(x)
        request.state.pose.position.y = float(y)
        request.state.pose.position.z = float(z)
        request.state.pose.orientation.w = 1.0
        request.state.reference_frame = 'world'
        self._client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = MeteorCraterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
