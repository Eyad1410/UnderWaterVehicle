import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time

class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')
        self.velocity_publisher = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.twist = TwistStamped()

    def move_vehicle(self, linear_x=0.5, duration=5.0):
        self.twist.twist.linear.x = linear_x
        self.twist.twist.linear.y = 0.0
        self.twist.twist.linear.z = 0.0
        self.twist.twist.angular.x = 0.0
        self.twist.twist.angular.y = 0.0
        self.twist.twist.angular.z = 0.0

        self.get_logger().info(f"Moving vehicle at {linear_x} m/s for {duration} seconds.")

        start_time = time.time()
        while time.time() - start_time < duration:
            self.twist.header.stamp = self.get_clock().now().to_msg()
            self.velocity_publisher.publish(self.twist)
            time.sleep(0.1)

        # Stop the vehicle
        self.stop_vehicle()

    def stop_vehicle(self):
        self.twist.twist.linear.x = 0.0
        self.twist.twist.linear.y = 0.0
        self.twist.twist.linear.z = 0.0
        self.twist.twist.angular.x = 0.0
        self.twist.twist.angular.y = 0.0
        self.twist.twist.angular.z = 0.0
        self.velocity_publisher.publish(self.twist)
        self.get_logger().info("Vehicle stopped.")

def main(args=None):
    rclpy.init(args=args)
    move_client = MoveClient()
    move_client.move_vehicle(linear_x=0.5, duration=3.0)
    move_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

