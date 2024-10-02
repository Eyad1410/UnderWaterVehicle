import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool
from std_srvs.srv import Empty

class ArmClient(Node):
    def __init__(self):
        super().__init__('arm_client')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')

    def arm_vehicle(self):
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /mavros/cmd/arming not available.')
            return False

        request = CommandBool.Request()
        request.value = True
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Vehicle armed successfully.')
                return True
            else:
                self.get_logger().warn('Failed to arm vehicle.')
                return False
        else:
            self.get_logger().error('Service call failed.')
            return False

def main(args=None):
    rclpy.init(args=args)
    arm_client = ArmClient()
    if arm_client.arm_vehicle():
        arm_client.get_logger().info("Vehicle is now armed.")
    else:
        arm_client.get_logger().info("Failed to arm the vehicle.")
    arm_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

