#! /usr/bin/env python 

import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
from std_msgs.msg import String
from leo_rover_tasks.plant_detector import PlantDetector
import cv2
from nav_msgs.msg import Odometry
from custom_interfaces.msg import RoverEvents
from pathlib import Path


class PlantDetectorNode(Node):
    def __init__(self):
        super().__init__('plant_detector_node')
        
        self.bridge = CvBridge()

        self.model_path = Path.home() / 'leo_ws' / 'src' / 'basic_ros2_extra_files' / 'plant_detector' / 'best_plant_detector_model.pth'


        self.plant_detector = PlantDetector(model_path=self.model_path)

        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.subscription
        self.current_odom = None

        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.odom_subscription

        self.publisher_ = self.create_publisher(String, '/plant_detector', 10)
        self.leo_rover_event_publisher_ = self.create_publisher(RoverEvents, "/leo_rover_events", 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        prediction = self.plant_detector.predict(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

        rover_event = RoverEvents()

        if prediction > 0.5:
            rover_event.info.data = f"Plant detected with confidence: {prediction:.2f}"
            self.get_logger().warning(rover_event.info.data)
            self.get_logger().warning("Publishing leo rover event...")
            rover_event.info.data = f"Plant detected with confidence: {prediction:.2f}"
            if self.current_odom:
                rover_event.rover_location = self.current_odom.pose.pose
            self.leo_rover_event_publisher_.publish(rover_event)
            result = f"Plant detected with confidence: {prediction:.2f}"
            self.get_logger().warning(result)
        else:
            rover_event.info.data = f"No plant detected. Confidence: {1 - prediction:.2f}"
            self.get_logger().info(rover_event.info.data)
            if self.current_odom:
                rover_event.rover_location = self.current_odom.pose.pose
            self.leo_rover_event_publisher_.publish(rover_event)
            result = f"No plant detected. Confidence: {1 - prediction:.2f}"
            self.get_logger().info(result)
        msg_string = String()
        msg_string.data = result
        self.publisher_.publish(msg_string)


    def odom_callback(self, msg):
        self.current_odom = msg

def main(args=None):
    rclpy.init(args=args)

    plant_detector_node = PlantDetectorNode()
    rclpy.spin(plant_detector_node)
    
    plant_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
