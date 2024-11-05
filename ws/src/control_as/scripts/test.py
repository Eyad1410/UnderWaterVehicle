#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.srv import CommandBool, SetMode

class TestCoordinates(Node):
    def __init__(self):
        super().__init__('test_coordinates')

        # Configure QoS for odometry subscription to match publisher's settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber for odometry to track position
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.odom_callback,
            qos_profile
        )

        # Publisher for position commands
        self.position_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Arm and set mode
        self.arm_rov()
        self.set_mode('GUIDED')

        # Target coordinates
        self.target_x = 5.0
        self.target_y = 5.0
        self.target_z = -2.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        # Timer to send position commands
        self.timer = self.create_timer(0.1, self.navigate_to_target)

    def odom_callback(self, msg):
        # Update current position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

    def arm_rov(self):
        client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        request = CommandBool.Request(value=True)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('ROV armed successfully')
        else:
            self.get_logger().info('Failed to arm the ROV')

    def set_mode(self, mode):
        client = self.create_client(SetMode, '/mavros/set_mode')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')
        request = SetMode.Request(custom_mode=mode)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f'Successfully set mode to {mode}')
        else:
            self.get_logger().info(f'Failed to set mode to {mode}')

    def navigate_to_target(self):
        # Command the ROV to move directly to target coordinates
        target_pose = PoseStamped()
        target_pose.header.frame_id = "map"
        target_pose.pose.position.x = self.target_x
        target_pose.pose.position.y = self.target_y
        target_pose.pose.position.z = self.target_z
        target_pose.pose.orientation.w = 1.0
        
        # Publish the target position
        self.position_publisher.publish(target_pose)

        # Log current position for feedback
        self.get_logger().info(f'Current Position: ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f})')

def main(args=None):
    rclpy.init(args=args)
    navigator = TestCoordinates()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

