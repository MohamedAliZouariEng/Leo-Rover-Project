# Import the Trigger module from std_srvs Service interface
from std_srvs.srv import Trigger
# Import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node

class RobotStatusClient(Node):

    def __init__(self):
        # Initialize the node with the name 'robot_status_client'
        super().__init__('robot_status_client')
        # Create the Service Client object
        # This defines the name ('/get_robot_status') and type (Trigger) of the Service Server to connect to.
        name_service = '/get_robot_status'
        self.client = self.create_client(Trigger, name_service)
        # Wait for the service to be available (checks every second)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service'+name_service+' not available, waiting again...')
        # Create an empty Trigger request
        self.req = Trigger.Request()

    def send_request(self):
        # Send the request asynchronously
        self.future = self.client.call_async(self.req)

    
def main(args=None):
    # Initialize the ROS communication
    rclpy.init(args=args)
    # Declare the node constructor
    client = RobotStatusClient()
    # Run the send_request() method
    client.send_request()
    while rclpy.ok():
        # Spin once to check for a service response
        rclpy.spin_once(client)
        if client.future.done():
            try:
                # Check if a response from the service was received
                response = client.future.result()
            except Exception as e:
                # Log any exceptions
                client.get_logger().info(f'Service call failed: {e}')
            else:
                # Log the service response
                client.get_logger().info(f'Success: {response.success}')
                client.get_logger().info(f'Status Report: {response.message}')
            break
    # Destroy the client node
    client.destroy_node()
    # Shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()