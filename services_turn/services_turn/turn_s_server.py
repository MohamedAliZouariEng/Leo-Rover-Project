#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from services_custom_pkg.srv import Turn
from geometry_msgs.msg import Twist
import time



class TurnNode(Node):
    def __init__(self):
        super().__init__('turn_server_node')
        # Create a service that will handle status queries
        name_service = '/turn'
        self.srv = self.create_service(Turn, name_service, self.turn_callback)

        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        
        self.get_logger().info(name_service+" Service Server Ready...")
    


    def turn_callback(self, request, response):
        twist_msg = Twist()
        if request.direction == 'left' :
            if request.angular_velocity:
                if request.time:
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = request.angular_velocity
                    self.publisher_.publish(twist_msg)
                    time.sleep(request.time)
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.0
                    self.publisher_.publish(twist_msg)
                    response.success = True
        elif request.direction == 'right':
            if request.angular_velocity:
                if request.time:
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = -1 * request.angular_velocity
                    self.publisher_.publish(twist_msg)
                    time.sleep(request.time)
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.0
                    self.publisher_.publish(twist_msg)
                    response.success = True
        else:
            response.success = False
            


        self.get_logger().info(f'Service called. Direction: {request.direction}\nAngular Velocity : {request.angular_velocity}\nTime : {request.time}, Success: {response.success}')
        return response
    
def main(args=None):
    rclpy.init(args=args)
    turn_service = TurnNode()
    try:
        rclpy.spin(turn_service)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    turn_service.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()