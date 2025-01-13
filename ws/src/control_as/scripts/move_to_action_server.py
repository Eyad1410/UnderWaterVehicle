#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool, SetMode
from rlab_customized_ros_msg.action import MoveTo
import math
import time


class MoveToActionServer(Node):
    def __init__(self):
        super().__init__('move_to_action_server')

        # Publishers and Subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.position_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.odom_callback,
            qos_profile
        )

        # ROV Initialization
        self.arm_rov()
        self.set_mode("GUIDED")

        # State Variables
        self.current_position = None
        self.target_position = None
        self.goal_reached_threshold = 0.05  # Distance threshold in meters
        self.goal_timeout = 3000.0  # Timeout in seconds

        # Action Server for MoveTo goals
        self.action_server = ActionServer(
            self,
            MoveTo,
            'move_to',
            execute_callback=self.execute_move_to_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info("MoveTo Action Server is ready to accept goals.")

    def odom_callback(self, msg):
        """Update the current position using odometry feedback."""
        self.current_position = msg.pose.pose.position
        self.get_logger().info(
            f"Current Position: x={self.current_position.x:.2f}, y={self.current_position.y:.2f}, z={self.current_position.z:.2f}"
        )

    def set_target_position(self, x, y, z):
        """Set a new target position."""
        self.target_position = PoseStamped()
        self.target_position.header.frame_id = "map"
        self.target_position.pose.position.x = x
        self.target_position.pose.position.y = y
        self.target_position.pose.position.z = z
        self.target_position.pose.orientation.w = 1.0

    def is_goal_reached(self):
        """Check if the ROV has reached the target position."""
        if not self.target_position or not self.current_position:
            return False
        dist_x = self.target_position.pose.position.x - self.current_position.x
        dist_y = self.target_position.pose.position.y - self.current_position.y
        dist_z = self.target_position.pose.position.z - self.current_position.z
        distance = math.sqrt(dist_x**2 + dist_y**2 + dist_z**2)
        self.get_logger().info(f"Distance to Target: {distance:.2f} meters")
        return distance < self.goal_reached_threshold

    def goal_callback(self, goal_request):
        """Handle incoming MoveTo action requests."""
        self.get_logger().info("MoveTo goal request received.")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle MoveTo action goal cancellation."""
        self.get_logger().info("MoveTo goal cancellation request received.")
        return CancelResponse.ACCEPT

    def execute_move_to_callback(self, goal_handle):
        """Execute MoveTo action goals."""
        target_pose = goal_handle.request.target_pose
        self.set_target_position(
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z
        )
        self.get_logger().info(
            f"Moving to Target: x={self.target_position.pose.position.x:.2f}, "
            f"y={self.target_position.pose.position.y:.2f}, z={self.target_position.pose.position.z:.2f}"
        )

        start_time = time.time()
        while not self.is_goal_reached():
            if goal_handle.is_cancel_requested:
                self.get_logger().info("MoveTo goal canceled.")
                goal_handle.abort()
                return MoveTo.Result(success=False)

            self.position_publisher.publish(self.target_position)  # Continuously publish target position
            rclpy.spin_once(self, timeout_sec=0.1)

            if time.time() - start_time > self.goal_timeout:
                self.get_logger().info("MoveTo goal timed out.")
                goal_handle.abort()
                return MoveTo.Result(success=False)

        self.get_logger().info("Goal reached successfully.")
        goal_handle.succeed()
        return MoveTo.Result(success=True)

    def arm_rov(self):
        """Arm the ROV."""
        client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for arming service...")
        request = CommandBool.Request(value=True)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info("ROV armed successfully.")
        else:
            self.get_logger().error("Failed to arm the ROV.")

    def set_mode(self, mode):
        """Set the ROV's mode."""
        client = self.create_client(SetMode, '/mavros/set_mode')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set_mode service...")
        request = SetMode.Request(custom_mode=mode)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f"Mode set to {mode}.")
        else:
            self.get_logger().error(f"Failed to set mode to {mode}.")


def main(args=None):
    rclpy.init(args=args)
    move_to_action_server = MoveToActionServer()
    rclpy.spin(move_to_action_server)
    move_to_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


