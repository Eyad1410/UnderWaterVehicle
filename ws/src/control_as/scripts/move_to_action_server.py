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
import json


class MoveToActionServer(Node):
    def __init__(self, json_file_path):
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
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.target_x = None
        self.target_y = None
        self.target_z = None
        self.goal_reached_threshold = 0.1  # Adjusted for closer accuracy
        self.goal_timeout = 30000.0

        # Load Positions from JSON File
        self.positions = self.load_positions(json_file_path)
        self.current_position_index = 0
        self.current_goal = None

        # Action Server for MoveTo goals
        self.action_server = ActionServer(
            self,
            MoveTo,
            'move_to',
            execute_callback=self.execute_move_to_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Timer for JSON-based goal processing
        self.timer = self.create_timer(0.1, self.process_json_goals)

    def load_positions(self, file_path):
        """Load positions from a JSON file."""
        try:
            with open(file_path, 'r') as file:
                data = json.load(file).get("positions", [])
                self.get_logger().info(f"Loaded {len(data)} positions from {file_path}")
                return data
        except Exception as e:
            self.get_logger().error(f"Failed to load positions from {file_path}: {e}")
            return []

    def odom_callback(self, msg):
        """Update the current position using odometry feedback."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

    def process_json_goals(self):
        """Handle JSON-driven goal execution."""
        if self.current_position_index >= len(self.positions):
            self.get_logger().info("All positions processed.")
            return

        # Check if the current goal is reached
        if self.current_goal and self.is_goal_reached():
            self.get_logger().info(f"Goal reached: {self.current_goal}")
            self.current_position_index += 1
            self.current_goal = None

        # Publish the next position if no active goal
        if not self.current_goal and self.current_position_index < len(self.positions):
            self.current_goal = self.positions[self.current_position_index]
            self.set_target_position(
                self.current_goal.get("x", 0.0),
                self.current_goal.get("y", 0.0),
                self.current_goal.get("z", 0.0)
            )

        # Continuously display current target and distance
        self.log_target_and_distance()

    def set_target_position(self, x, y, z):
        """Set a new target position."""
        self.target_x = x
        self.target_y = y
        self.target_z = z
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self.position_publisher.publish(pose)

    def is_goal_reached(self):
        """Check if the ROV has reached the target position."""
        if self.target_x is None or self.target_y is None or self.target_z is None:
            return False
        dist_x = self.target_x - self.current_x
        dist_y = self.target_y - self.current_y
        dist_z = self.target_z - self.current_z
        distance = math.sqrt(dist_x**2 + dist_y**2 + dist_z**2)
        return distance < self.goal_reached_threshold

    def log_target_and_distance(self):
        """Continuously log the target position and distance."""
        if self.target_x is not None and self.target_y is not None and self.target_z is not None:
            dist_x = self.target_x - self.current_x
            dist_y = self.target_y - self.current_y
            dist_z = self.target_z - self.current_z
            distance = math.sqrt(dist_x**2 + dist_y**2 + dist_z**2)
            self.get_logger().info(
                f"Current Target: x={self.target_x}, y={self.target_y}, z={self.target_z} | "
                f"Distance to Target: {distance:.2f} meters"
            )

    def goal_callback(self, goal_request):
        """Handle incoming MoveTo action requests."""
        self.get_logger().info("MoveTo goal request received.")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle MoveTo action goal cancellation."""
        self.get_logger().info("MoveTo goal cancellation request received.")
        goal_handle.cancel()
        return CancelResponse.ACCEPT

    def execute_move_to_callback(self, goal_handle):
        """Execute MoveTo action goals."""
        target_pose = goal_handle.request.target_pose
        self.target_x = target_pose.pose.position.x
        self.target_y = target_pose.pose.position.y
        self.target_z = target_pose.pose.position.z
        self.get_logger().info(f"Executing MoveTo goal: x={self.target_x}, y={self.target_y}, z={self.target_z}")

        start_time = time.time()
        while not self.is_goal_reached():
            self.log_target_and_distance()
            self.publish_target_position()
            rclpy.spin_once(self, timeout_sec=0.1)

            if time.time() - start_time > self.goal_timeout:
                self.get_logger().info("MoveTo goal timed out.")
                goal_handle.abort()
                return MoveTo.Result()

        goal_handle.succeed()
        self.get_logger().info("MoveTo goal completed successfully.")
        return MoveTo.Result()

    def publish_target_position(self):
        """Continuously publish the target position."""
        if self.target_x is not None and self.target_y is not None and self.target_z is not None:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = self.target_x
            pose.pose.position.y = self.target_y
            pose.pose.position.z = self.target_z
            pose.pose.orientation.w = 1.0
            self.position_publisher.publish(pose)

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
            self.get_logger().info("Failed to arm the ROV.")

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
            self.get_logger().info(f"Failed to set mode to {mode}.")


def main(args=None):
    rclpy.init(args=args)
    json_file_path = "/UnderWaterVehicle/ws/src/control_as/scripts/positions.json"
    move_to_action_server = MoveToActionServer(json_file_path)
    rclpy.spin(move_to_action_server)
    move_to_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


