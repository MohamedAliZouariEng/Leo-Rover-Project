#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import torch.nn as nn
from torchvision import transforms, models
from PIL import Image as PILImage
from geometry_msgs.msg import Twist # Import for velocity commands
from rclpy.duration import Duration
import time
from std_srvs.srv import Trigger # Import the Trigger service
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from pathlib import Path

class PlantDetectorNode(Node):
    def __init__(self, image_topic_name = "/leo/camera/image_raw"):
        super().__init__('plant_detector_node')
        self.mutuallyexclusive_group_1 = MutuallyExclusiveCallbackGroup()
        self.mutuallyexclusive_group_2 = MutuallyExclusiveCallbackGroup()
        self._image_topic_name = image_topic_name
        self.subscription = self.create_subscription(
            Image,
            self._image_topic_name,
            self.listener_callback,
            10,
            callback_group=self.mutuallyexclusive_group_1
        )
        self.subscription # prevent unused variable warning
        self.bridge = CvBridge()
        self.last_image = None
        # Publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # Load the trained plant detection model
        self.model = models.resnet18(weights=None)
        self.model.fc = nn.Sequential(
        nn.Linear(self.model.fc.in_features, 512),
        nn.ReLU(inplace=True),
        nn.Dropout(0.5),
        nn.Linear(512, 1)
        )
        self.model_path = Path.home() / 'leo_ws' / 'src' / 'basic_ros2_extra_files' / 'plant_detector' / 'best_alien_plant_detector_model.pth'


        #self.model_path = "/home/user/ubuntu24/src/basic_ros2_extra_files/plant_detector/best_alien_plant_detector_model.pth"
        self.model.load_state_dict(torch.load(self.model_path, map_location=torch.device('cpu')))
        self.model.eval()
        # Define image preprocessing steps
        self.transform = transforms.Compose([
            transforms.Resize((150, 150)),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])
        # Create the service for plant detection
        self.srv = self.create_service(Trigger, 'detect_plants', self.detect_plants_callback, callback_group=self.mutuallyexclusive_group_2)


    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        self.last_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def detect_plants_callback(self, request, response):
        start_time = self.get_clock().now()
        duration_10_sec = Duration(seconds=10)
        plant_found = False
        # Start moving the robot
        self.publish_velocity(linear=0.0, angular=-0.5)
        delta_time = self.get_clock().now() - start_time
        while delta_time < duration_10_sec and not plant_found:
            delta_time = self.get_clock().now() - start_time
            if self.last_image is not None:
                self.get_logger().info("Processing Image for Plants...")
                # Convert OpenCV image (BGR) to PIL Image (RGB)
                pil_image = PILImage.fromarray(cv2.cvtColor(self.last_image, cv2.COLOR_BGR2RGB))
                # Preprocess the image
                input_tensor = self.transform(pil_image).unsqueeze(0)
                # Run the model to detect plants
                with torch.no_grad():
                    output = self.model(input_tensor)
                    prediction = torch.sigmoid(output).item()
                if prediction > 0.5:
                    self.publish_velocity(linear=0.0, angular=0.0)
                    self.get_logger().info(f"Plant Detected with Confidence: {prediction:.2f}")
                    plant_found = True
                else:
                    self.get_logger().info(f"No Plant Detected. Confidence: {1 - prediction:.2f}")
            time.sleep(0.1)
        # Stop the robot
        self.publish_velocity(linear=0.0, angular=0.0)
        response.success = plant_found
        response.message = 'Plant detection completed'
        return response
    

    def publish_velocity(self, linear, angular):
        # Create Twist message
        vel_msg = Twist()
        vel_msg.linear.x = linear
        vel_msg.angular.z = angular
        # Publish the message
        self.cmd_vel_publisher.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    plant_detector_node = PlantDetectorNode()
    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(plant_detector_node)
    try:
        executor.spin()
    finally:
        plant_detector_node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()