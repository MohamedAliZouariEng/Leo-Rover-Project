#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger # Using Trigger for simple status checking
from random import uniform, choice

class RobotStatusService(Node):
    def __init__(self):
        super().__init__('robot_status_service')
        # Create a service that will handle status queries
        name_service = '/get_robot_status'
        self.srv = self.create_service(Trigger, name_service, self.get_status_callback)
        self.get_logger().info(name_service+" Service Server Ready...")

    def get_status_callback(self, request, response):
        # Simulated data
        battery_level = round(uniform(20.0, 100.0), 2) # Simulated battery percentage
        temperature = round(uniform(25.0, 80.0), 2)
        # Simulated temperature in Celsius
        camera_status = choice([True, False])
        # Camera status
        motor_statuses = [choice([True, False]) for _ in range(4)] # Status of 4 motors
        # Construct response message
        response.success = True
        response.message = (
            f"Battery Level: {battery_level}%, "
            f"Temperature: {temperature}°C, "
            f"Camera: {'Operational' if camera_status else 'Faulty'}, "
            f"Motor 1: {'Operational' if motor_statuses[0] else 'Faulty'}, "
            f"Motor 2: {'Operational' if motor_statuses[1] else 'Faulty'}, "
            f"Motor 3: {'Operational' if motor_statuses[2] else 'Faulty'}, "
            f"Motor 4: {'Operational' if motor_statuses[3] else 'Faulty'}, "
        )
        return response
    
def main(args=None):
    rclpy.init(args=args)
    robot_status_service = RobotStatusService()
    try:
        rclpy.spin(robot_status_service)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    robot_status_service.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()