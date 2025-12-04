import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import torch.nn as nn
from torchvision import transforms, models
from PIL import Image as PILImage
import numpy as np

class PlantDetectorNode(Node):
    def __init__(self):
        super().__init__('plant_detector_node')
        
        # Initialize the CvBridge
        self.bridge = CvBridge()
        
        # Subscribe to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Load the trained model
        self.model = models.resnet18(weights=None)
        self.model.fc = nn.Sequential(
            nn.Linear(self.model.fc.in_features, 512),
            nn.ReLU(inplace=True),
            nn.Dropout(0.5),
            nn.Linear(512, 1)
        )
        self.model.load_state_dict(torch.load('best_alien_plant_detector_model.pth', map_location=torch.device('cpu')))
        self.model.eval()
        
        # Define image preprocessing
        self.transform = transforms.Compose([
            transforms.Resize((150, 150)),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Convert OpenCV image (BGR) to PIL Image (RGB)
        pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        
        # Preprocess the image
        input_tensor = self.transform(pil_image).unsqueeze(0)
        
        # Run the model to detect a plant
        with torch.no_grad():
            output = self.model(input_tensor)
            prediction = torch.sigmoid(output).item()
        
        # Log information based on the prediction
        if prediction > 0.5:
            self.get_logger().warning(f"Plant detected with confidence: {prediction:.2f}")
        else:
            self.get_logger().info(f"No plant detected. Confidence: {1 - prediction:.2f}")

def main(args=None):
    rclpy.init(args=args)

    plant_detector_node = PlantDetectorNode()

    rclpy.spin(plant_detector_node)

    # Destroy the node explicitly (optional)
    plant_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
