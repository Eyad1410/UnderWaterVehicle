import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Twist


class ArmAndMoveClient(Node):
    def __init__(self):
        super().__init__('arm_and_move_client')
        # Create a client to arm the vehicle
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        # Create a client to set the mode
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        # Create a publisher to send velocity commands
        self.vel_publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

    def arm_vehicle(self):
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /mavros/cmd/arming not available.')
            return False

        request = CommandBool.Request()
        request.value = True
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info('Vehicle armed successfully.')
            return True
        else:
            self.get_logger().error('Failed to arm the vehicle.')
            return False

    def set_guided_mode(self):
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /mavros/set_mode not available.')
            return False

        request = SetMode.Request()
        request.custom_mode = 'GUIDED'
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info('GUIDED mode set successfully.')
            return True
        else:
            self.get_logger().error('Failed to set GUIDED mode.')
            return False

    def move_forward(self):
        self.get_logger().info("Moving the vehicle forward.")

        # Create a velocity command to move forward
        twist_msg = Twist()
        twist_msg.linear.x = 1.0  # Move forward with speed 1.0 m/s
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        # Publish the velocity command
        self.vel_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)

    client = ArmAndMoveClient()

    # Arm the vehicle
    if client.arm_vehicle():
        client.get_logger().info("Vehicle is armed, setting to GUIDED mode...")

        # Set to GUIDED mode
        if client.set_guided_mode():
            client.get_logger().info("Vehicle is in GUIDED mode, moving forward...")
            client.move_forward()
        else:
            client.get_logger().info("Failed to set vehicle to GUIDED mode.")
    else:
        client.get_logger().info("Failed to arm the vehicle.")

    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

