import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class AutonomousROVClient(Node):
    def __init__(self):
        super().__init__('autonomous_rov_client')

    def send_snail_pattern_request(self):
        client = self.create_client(Trigger, 'execute_snail_pattern')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for execute_snail_pattern service...')
        
        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('Snail pattern execution successful')
        else:
            self.get_logger().info('Snail pattern execution failed')

    def send_move_forward_request(self):
        client = self.create_client(Trigger, 'move_forward')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_forward service...')
        
        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('Move forward execution successful')
        else:
            self.get_logger().info('Move forward execution failed')

def main(args=None):
    rclpy.init(args=args)
    client = AutonomousROVClient()
    client.send_snail_pattern_request()  # Request to execute the snail pattern
    client.send_move_forward_request()   # Request to move forward
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

