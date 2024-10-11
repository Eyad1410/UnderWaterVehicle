import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Twist
import time


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
        self.get_logger().info('Waiting for /mavros/cmd/arming service...')
        if not self.arm_client.wait_for_service(timeout_sec=10.0):
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
        self.get_logger().info('Waiting for /mavros/set_mode service...')
        if not self.set_mode_client.wait_for_service(timeout_sec=10.0):
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

    def move_forward(self, duration=5):
        self.get_logger().info("Moving the vehicle forward for {} seconds.".format(duration))

        # Wait for the publisher to be ready
        while not self.vel_publisher.get_subscription_count():
            self.get_logger().warn('Waiting for subscriber...')
            time.sleep(1.0)

        # Create a velocity command to move forward
        twist_msg = Twist()
        twist_msg.linear.x = 0.5  # Move forward with speed 0.5 m/s

        start_time = time.time()
        while time.time() - start_time < duration:
            self.vel_publisher.publish(twist_msg)
            time.sleep(0.1)  # Adjust the rate at which commands are published

        # Stop the movement after the duration
        self.get_logger().info("Stopping the vehicle.")
        twist_msg.linear.x = 0.0
        self.vel_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)

    client = ArmAndMoveClient()

    # Wait for everything to initialize (simulator or vehicle)
    time.sleep(5)  # Give the simulator time to initialize

    # Arm the vehicle
    if client.arm_vehicle():
        client.get_logger().info("Vehicle is armed, setting to GUIDED mode...")

        # Set to GUIDED mode
        if client.set_guided_mode():
            client.get_logger().info("Vehicle is in GUIDED mode, moving forward...")
            client.move_forward(duration=5)
        else:
            client.get_logger().info("Failed to set vehicle to GUIDED mode.")
    else:
        client.get_logger().info("Failed to arm the vehicle.")

    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

