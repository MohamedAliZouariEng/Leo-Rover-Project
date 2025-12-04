#! /usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2
from leo_rover_tasks.text_recog import TextRecognition
from rclpy.qos import ReliabilityPolicy, QoSProfile
from pathlib import Path


class TextRecorgNode(Node):
    def __init__(self):
        super().__init__("text_recog_node")
        self.bridge = CvBridge()

        self.subscriber = self.create_subscription(Image, 
                                                   "/camera/image_raw",
                                                    self.image_callback,
                                                    QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().info("text_recog_node Ready...")
        
        east_model_path = Path.home() / 'leo_ws' / 'src' / 'basic_ros2_extra_files' / 'text_detector' / 'frozen_east_text_detection.pb'
        #east_model_path = '/home/ubuntu24/leo_ws/src/basic_ros2_extra_files/text_detector/frozen_east_text_detection.pb'
        min_confidence = 0.5
        width = 320
        height = 320
        padding = 0.0
        self.text_recognizer = TextRecognition(east_model_path, min_confidence, width, height, padding)
        
        
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        results = self.text_recognizer.recognize_text(cv_image)

        for (start_x, start_y, end_x, end_y), text in results:
            # Detections have these elements at the end
            cleaned_text = text.rstrip(".\n\x0c")
            position = str(start_x)+"-"+str(start_y)+"-"+str(end_x)+"-"+str(end_y)
            self.get_logger().info(f'OCR Result: {cleaned_text}, ({position})')
        

def main(args=None):
    rclpy.init(args=args)
    node = TextRecorgNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()