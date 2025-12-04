#! /usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist 
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import tf_transformations
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class SpecificLocationNode(Node):
    def __init__(self):
        super().__init__("specific_location_node")
        self.subscriber = self.create_subscription(LaserScan, 
                                                   "/scan",
                                                    self.laserscan_callback,
                                                    QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info("specific_location_node Ready...")

        self.odom_sub = self.create_subscription(Odometry,
                                                 "/odom",
                                                 self.odom_callback,
                                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        self.subscriber_location = self.create_subscription(String, '/location_mission', self.location_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.status_publisher_ = self.create_publisher(String, '/location_mission_pub', 10)
        
        # Initialize state variables
        self.turning = False
        self.turn_direction = -0.5 # Default to turning right
        self.distance_from_origin = 0.0
        self.returning_to_origin = False
        self.current_position = {'x': 0.0, 'y': 0.0} # Store the current x and y position
        self.home_position = {'x': 0.0, 'y': 0.0}
        self.pickup_position = {'x': -2.342, 'y': -2.432}
        self.current_goal = None
        self.goal_tolerance = 0.1
        self.mission_active = False
        self.yaw = 0.0 # Yaw angle of the robot
        self.get_logger().info("Specific Location Node Ready...")

    def location_callback(self, msg):
        if msg.data == 'Go-Home':
            self.mission_active = True
            self.current_goal = self.home_position
        elif msg.data == 'Go-Pickup':
            self.mission_active = True
            self.current_goal = self.pickup_position
        else:
            self.get_logger().warning('!!! Hey Man !!!, You have only two options  Go-Home or Go-Pickup')

    def laserscan_callback(self, msg):
        if self.mission_active:
            twist_msg = Twist()

            # Define the sectors
            sectors = {
                "Right_Rear": (0, 33),
                "Right": (34, 66),
                "Front_Right": (67, 100),
                "Front_Left": (101, 133),
                "Left": (134, 166),
                "Left_Rear": (167, 199)
            }
            # Initialize the minimum distances for each sector
            min_distances = {key: float('inf') for key in sectors.keys()}
            # Find the minimum distance in each sector
            for sector, (start_idx, end_idx) in sectors.items():
                # Ensure the index range is within bounds and not empty
                if start_idx < len(msg.ranges) and end_idx < len(msg.ranges):
                    sector_ranges = msg.ranges[start_idx:end_idx + 1]
                    if sector_ranges:
                        min_distances[sector] = min(sector_ranges)
            # Log the minimum distances
            for sector, min_distance in min_distances.items():
                self.get_logger().info(f'{sector}: {min_distance:.2f} meters')
            # Define the threshold for obstacle detection
            obstacle_threshold = 0.8 # meters
            # Determine detected obstacles
            detections = {sector: min_distance < obstacle_threshold for sector, min_distance in min_distances.items()}
            # Determine suggested action based on detection and ordered Cinditions by priority
            # Priority 1: Front detection, Priority 2: Side detections, Priority 3: Rear detections
            # Priority 1
            if detections["Front_Left"] and detections["Front_Right"]:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.5
                
            elif detections["Front_Left"] and not detections["Front_Right"]:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = -0.5
            elif detections["Front_Right"] and not detections["Front_Left"]:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.5
            # Priority 2
            elif detections["Left"]:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = -0.5
            elif detections["Right"]:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.5
            else:
                twist_msg.linear.x = 0.3
                twist_msg.angular.z = 0.0
            # Log the suggested action
            self.publisher_.publish(twist_msg)

        
    def odom_callback(self,msg):
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y
        # Calculate the distance from the origin (0,0)
        self.distance_from_origin = math.sqrt(self.current_position['x']**2 + self.current_position['y']**2)
        self.get_logger().info(f'Distance from origin: {self.distance_from_origin:.2f} meters')
        # Calculate the yaw (orientation around the z-axis)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = tf_transformations.euler_from_quaternion(orientation_list)

        # Check if the goal has been reached
        if self.current_goal:
            distance_to_goal = math.sqrt(
                (self.current_position['x'] - self.current_goal['x'])**2 +
                (self.current_position['y'] - self.current_goal['y'])**2
            )
            self.go_to_target()
            if distance_to_goal < self.goal_tolerance:
                self.stop_robot()
                self.mission_active = False
                self.current_goal = None
                self.get_logger().info("Goal reached. Robot stopped.")
                self.status_publisher_.publish(String(data='goal-reached'))
   
    def is_at_position(self, target_position):
        """Check if the robot is within the tolerance of a target position."""
        distance_to_target = math.sqrt(
            (self.current_position['x'] - target_position['x'])**2 +
            (self.current_position['y'] - target_position['y'])**2
        )
        return distance_to_target < self.goal_tolerance
    
    def go_to_target(self):
        action = Twist()
        # Calculate the desired angle to the origin
        desired_yaw = math.atan2(self.current_goal['y'] - self.current_position['y'], self.current_goal['x'] - self.current_position['x'])
        # Calculate the difference between current yaw and desired yaw
        yaw_error = desired_yaw - self.yaw
        # Normalize the yaw error to the range [-pi, pi]
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi
        # If the yaw error is significant, rotate towards the origin
        if abs(yaw_error) > 0.1: # 0.1 radians threshold for orientation
            action.angular.z = 0.5 if yaw_error > 0 else -0.5
            self.get_logger().info(f'Turning towards target. Yaw error: {yaw_error:.2f}')
        else:
            # If oriented towards the origin, move forward
            action.linear.x = 0.5
            self.get_logger().info('Heading towards target.')
        self.publisher_.publish(action)

    def stop_robot(self):
        msg = Twist()
        self.publisher_.publish(msg)





def main(args=None):
    rclpy.init(args=args)
    node = SpecificLocationNode()
    rclpy.spin_once(node)
    # Detect the robot's initial position and decide where to move
    rclpy.spin_once(node) # Process initial odometry
    if not node.is_at_position(node.home_position) and not node.is_at_position(node.pickup_position):
        node.get_logger().info("Initial position not at home or pickup. Moving to home position.")
        node.current_goal = node.home_position
        node.mission_active = True
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()