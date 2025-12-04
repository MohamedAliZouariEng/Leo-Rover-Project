#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_interfaces.srv import TextRecognitionSrv


from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2
from leo_rover_tasks.text_recog import TextRecognition
from rclpy.qos import ReliabilityPolicy, QoSProfile
from pathlib import Path


class TextRecognitionServiceCustom(Node):
    def __init__(self):
        super().__init__('text_recognition_service_custom')
        # Create a service that will handle status queries
        name_service = '/text_recognition_service_custom'
        self.srv = self.create_service(TextRecognitionSrv, name_service, self.text_recognition_server_callback)
        self.bridge = CvBridge()
        self.last_detected_text = ''
        self.position = ''
        self.start_x = 0
        self.start_y = 0
        self.end_x = 0
        self.end_y = 0

        self.subscriber = self.create_subscription(Image, 
                                                   "/camera/image_raw",
                                                    self.image_callback,
                                                    QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        east_model_path = Path.home() / 'leo_ws' / 'src' / 'basic_ros2_extra_files' / 'text_detector' / 'frozen_east_text_detection.pb'
        
        #east_model_path = '/home/ubuntu24/leo_ws/src/basic_ros2_extra_files/text_detector/frozen_east_text_detection.pb'
        min_confidence = 0.5
        width = 320
        height = 320
        padding = 0.0
        self.text_recognizer = TextRecognition(east_model_path, min_confidence, width, height, padding)
        
        self.get_logger().info(name_service+" Service Server Ready...")
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        results = self.text_recognizer.recognize_text(cv_image)
        if results:
            for (start_x, start_y, end_x, end_y), text in results:
                # Detections have these elements at the end
                self.start_x = start_x
                self.start_y = start_y
                self.end_x = end_x
                self.end_y = end_y
                self.position = str(start_x)+"-"+str(start_y)+"-"+str(end_x)+"-"+str(end_y)

                cleaned_text = text.rstrip(".\n\x0c")
                self.last_detected_text = cleaned_text
                break # Only keep the first recognized text
        else:
            self.last_detected_text = ''
        self.get_logger().info(f'Result: {self.last_detected_text}')

    def text_recognition_server_callback(self, request, response):
        # Construct response message
        detected_text = self.last_detected_text.upper()
        if detected_text == request.label:
            response.success = True
            response.start_x = self.start_x
            response.start_y = self.start_y
            response.end_x = self.end_x
            response.end_y = self.end_y
        else:
            response.success = False
            response.start_x = 0
            response.start_y = 0
            response.end_x = 0
            response.end_y = 0
        

        self.get_logger().info(f'Service called. Position: {self.position}, Success: {response.success}')
        return response
    
def main(args=None):
    rclpy.init(args=args)
    text_recognition_service = TextRecognitionServiceCustom()
    try:
        rclpy.spin(text_recognition_service)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    text_recognition_service.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()