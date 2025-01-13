#!/usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from mavros_msgs.srv import SetMode, CommandBool
from rclpy.node import Node
from rlab_customized_ros_msg.action import SnailPattern
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math
from rclpy.duration import Duration
import time


class AutonomousROVController(Node):
    def __init__(self):
        super().__init__('autonomous_rov_controller')

        # Define callback groups
        self.snail_pattern_callback_group = MutuallyExclusiveCallbackGroup()
        self.odom_callback_group = MutuallyExclusiveCallbackGroup()

        # Action Server for SnailPattern
        self._snail_pattern_action_server = ActionServer(
            self,
            SnailPattern,
            'snail_pattern',
            callback_group=self.snail_pattern_callback_group,
            execute_callback=self.execute_snail_pattern_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Publishers and Subscribers
        self.position_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.odom_callback,
            qos_profile,
            callback_group=self.odom_callback_group
        )

        # State Variables
        self.current_position = None
        self.start_position = None
        self.current_yaw = 0.0
        self.goal_active = False
        self.canceled = False
        self.current_goal_handle = None

        # Initialize ROV
        self.arm_rov()
        self.set_mode('GUIDED')
        self.get_logger().info('Autonomous ROV Controller is ready and waiting for goals.')

    def goal_callback(self, goal_request):
        """Accept incoming goals."""
        self.get_logger().info('Goal accepted.')
        self.goal_active = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle goal cancellation."""
        self.get_logger().info('Goal cancellation request received.')
        self.safe_abort()
        return CancelResponse.ACCEPT

    def odom_callback(self, msg):
        """Update the current position from odometry."""
        self.current_position = msg.pose.pose.position

    def execute_snail_pattern_callback(self, goal_handle):
        """Execute the SnailPattern goal."""
        self.current_goal_handle = goal_handle
        try:
            self.get_logger().info('Executing SnailPattern.')
            goal_request = goal_handle.request

            success = self.execute_snail_pattern(
                goal_handle,
                initial_side_length=goal_request.initial_side_length,
                increment=goal_request.increment,
                max_side_length=goal_request.max_side_length
            )

            if success:
                goal_handle.succeed()
                self.get_logger().info('Goal succeeded.')
            else:
                goal_handle.abort()
                self.get_logger().info('Goal failed.')

        except Exception as e:
            self.get_logger().error(f"Error during execution: {e}")
        finally:
            self.get_logger().info("Waiting for the next goal...")
            self.goal_active = False

        # Return result
        result = SnailPattern.Result()
        result.success = success
        return result

    def execute_snail_pattern(self, goal_handle, initial_side_length, increment, max_side_length):
        """Execute the snail pattern."""
        current_side_length = initial_side_length
        yaw_angles = [0, -90, -180, -270]
        move_count = 0

        while current_side_length <= max_side_length and rclpy.ok():
            for yaw in yaw_angles:
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal canceled during execution.')
                    return False

                # Rotate to target yaw
                if not self.rotate_to_yaw(math.radians(yaw)):
                    return False

                # Move forward for the current side length
                if not self.move_forward(current_side_length):
                    return False

                # Publish progress feedback
                move_count += 1
                progress = (current_side_length / max_side_length) * 100
                feedback = SnailPattern.Feedback()
                feedback.progress = progress
                goal_handle.publish_feedback(feedback)
                self.get_logger().info(f'Snail pattern progress: {progress:.2f}%')

                # Increment the side length every two sides
                if move_count % 2 == 0:
                    current_side_length += increment
                    if current_side_length > max_side_length:
                        break
                    self.get_logger().info(f'Increasing side length to {current_side_length:.2f} meters.')

        return True

    def rotate_to_yaw(self, target_yaw, max_duration=10.0):
        """Rotate the ROV to the desired yaw angle."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        start_time = self.get_clock().now()

        while (self.get_clock().now() - start_time) < Duration(seconds=max_duration) and rclpy.ok():
            # Publish the rotation command
            pose.pose.orientation.w = math.cos(target_yaw / 2)
            pose.pose.orientation.z = math.sin(target_yaw / 2)
            pose.header.stamp = self.get_clock().now().to_msg()
            self.position_publisher.publish(pose)

            # Simulate small delay
            time.sleep(0.1)

        self.current_yaw = target_yaw
        self.stabilize_after_action(2.0)
        return True

    def move_forward(self, distance):
        """Move the ROV forward by the specified distance."""
        velocity = Twist()
        self.start_position = self.current_position

        while self.get_traveled_distance() < distance and rclpy.ok():
            velocity.linear.x = 1.0 * math.cos(self.current_yaw)
            velocity.linear.y = 1.0 * math.sin(self.current_yaw)
            self.velocity_publisher.publish(velocity)
            time.sleep(0.1)

        self.stop_movement()
        self.stabilize_after_action(2.0)
        return True

    def get_traveled_distance(self):
        """Calculate the traveled distance."""
        if self.start_position and self.current_position:
            dx = self.current_position.x - self.start_position.x
            dy = self.current_position.y - self.start_position.y
            return math.sqrt(dx**2 + dy**2)
        return 0.0

    def stop_movement(self):
        """Stop the ROV."""
        self.velocity_publisher.publish(Twist())

    def stabilize_after_action(self, duration):
        """Stabilize the ROV after movement."""
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time) < Duration(seconds=duration) and rclpy.ok():
            self.velocity_publisher.publish(Twist())
            time.sleep(0.1)

    def arm_rov(self):
        """Arm the ROV."""
        client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        request = CommandBool.Request(value=True)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('ROV armed successfully.')
        else:
            self.get_logger().error('Failed to arm the ROV.')

    def set_mode(self, mode):
        """Set the ROV mode."""
        client = self.create_client(SetMode, '/mavros/set_mode')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')
        request = SetMode.Request(custom_mode=mode)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f'Mode set to {mode}')
        else:
            self.get_logger().error(f'Failed to set mode to {mode}')


def main(args=None):
    rclpy.init(args=args)
    autonomous_rov_controller = AutonomousROVController()
    executor = MultiThreadedExecutor()
    executor.add_node(autonomous_rov_controller)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()



