#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np



class GreenDetector(Node):
    def __init__(self):
        super().__init__('green_detector')
        # Create a subscriber to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
        10)
        self.bridge = CvBridge()
        # Create windows with trackbars for adjusting HSV values
        cv2.namedWindow("Green Detection")
        cv2.createTrackbar("Lower H", "Green Detection", 0, 179, self.nothing)
        cv2.createTrackbar("Lower S", "Green Detection", 0, 255, self.nothing)
        cv2.createTrackbar("Lower V", "Green Detection", 0, 255, self.nothing)
        cv2.createTrackbar("Upper H", "Green Detection", 179, 179, self.nothing)
        cv2.createTrackbar("Upper S", "Green Detection", 255, 255, self.nothing)
        cv2.createTrackbar("Upper V", "Green Detection", 255, 255, self.nothing)

    def image_callback(self, msg):
        # Convert the ROS image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Get current positions of all trackbars
        lower_h = cv2.getTrackbarPos("Lower H", "Green Detection")
        lower_s = cv2.getTrackbarPos("Lower S", "Green Detection")
        lower_v = cv2.getTrackbarPos("Lower V", "Green Detection")
        upper_h = cv2.getTrackbarPos("Upper H", "Green Detection")
        upper_s = cv2.getTrackbarPos("Upper S", "Green Detection")
        upper_v = cv2.getTrackbarPos("Upper V", "Green Detection")
        # Set the lower and upper bounds for HSV
        lower_green = np.array([lower_h, lower_s, lower_v])
        upper_green = np.array([upper_h, upper_s, upper_v])
        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Create a mask to threshold the image
        mask = cv2.inRange(hsv_image, lower_green, upper_green)
        # Optionally visualize the green detection on the original image
        green_detected = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        # Display the original image and the mask
        cv2.imshow("Green Detection", green_detected)
        # Press 'q' to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy()
    def destroy(self):
        cv2.destroyAllWindows()
        self.destroy_node()


        
def main(args=None):
    rclpy.init(args=args)
    # Create a green detector node
    green_detector = GreenDetector()
    try:
        rclpy.spin(green_detector)
    except KeyboardInterrupt:
        green_detector.destroy()
        # Clean up
        rclpy.shutdown()


if __name__ == '__main__':
    main()