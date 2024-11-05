#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from mavros_msgs.srv import SetMode, CommandBool
from rclpy.node import Node
from rlab_customized_ros_msg.action import SnailPattern
from std_msgs.msg import Header
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math
from rclpy.duration import Duration

class AutonomousROVController(Node):
    def __init__(self):
        super().__init__('autonomous_rov_controller')

        self._action_server = ActionServer(
            self,
            SnailPattern,
            'snail_pattern',
            callback_group=ReentrantCallbackGroup(),
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.position_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # Configure QoS for odometry subscription to match publisher's settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.odom_callback,
            qos_profile
        )
        self.current_position = None
        self.start_position = None

        self.arm_rov()
        self.set_mode('GUIDED')
        
        # Initialize yaw tracking variables
        self.current_yaw = 0.0
        self.target_yaw = 0.0

        # Initialize tracking variables
        self.reset_goal_state()
        self.get_logger().info('autonomous_rov_controller is ready and will stay active for repeated goals')

    def reset_goal_state(self):
        """Resets internal state flags and goal handles."""
        self.goal_active = False
        self.canceled = False
        self.current_goal_handle = None

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        
        # Cancel any current goal if one is active
        if self.goal_active and self.current_goal_handle:
            self.get_logger().info('Canceling previous goal')
        
        self.reset_goal_state()  # Clear state for new goal
        self.goal_active = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        self.canceled = True  # Track cancellation
        return CancelResponse.ACCEPT

    def odom_callback(self, msg):
        """Callback to update current position from odometry data."""
        self.current_position = msg.pose.pose.position

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing SnailPattern')
        self.current_goal_handle = goal_handle

        # Early cancellation check
        if goal_handle.is_cancel_requested:
            self.get_logger().info('Goal was canceled before execution started')
            goal_handle.canceled()
            self.reset_goal_state()
            return self.canceled_result()

        # Execute snail pattern
        goal_request = goal_handle.request
        initial_side_length = goal_request.initial_side_length
        increment = goal_request.increment
        max_side_length = goal_request.max_side_length
        success = self.execute_snail_pattern(goal_handle, initial_side_length, increment, max_side_length)

        # Final cancellation check before concluding execution
        if goal_handle.is_cancel_requested:
            self.get_logger().info('Goal was canceled at the end of execution')
            goal_handle.canceled()
            self.reset_goal_state()
            return self.canceled_result()

        # Mark as succeeded if successful, or handle failure explicitly
        if success:
            goal_handle.succeed()
            self.get_logger().info('Goal succeeded')
        else:
            # Log failure and ensure a full reset
            self.get_logger().info('Goal failed to reach target')
            self.handle_failed_goal()  # Explicit handling of a failed goal

        # Reset goal state to keep server ready for the next goal
        self.reset_goal_state()
        return SnailPattern.Result(success=success)

    def handle_failed_goal(self):
        """Explicit handling after a goal fails to reset and return to idle state."""
        self.get_logger().info('Handling failed goal and resetting state.')
        self.stop_movement()  # Ensure any active movement is stopped
        self.stabilize_after_action(duration=2.0)  # Stabilize to neutral position
        self.reset_goal_state()  # Complete reset

    def canceled_result(self):
        result = SnailPattern.Result()
        result.success = False
        return result

    def arm_rov(self):
        arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not arm_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        
        request = CommandBool.Request(value=True)
        future = arm_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('ROV armed successfully')
        else:
            self.get_logger().info('Failed to arm the ROV')

    def set_mode(self, mode):
        mode_service = self.create_client(SetMode, '/mavros/set_mode')
        while not mode_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')
        
        request = SetMode.Request(custom_mode=mode)
        future = mode_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().mode_sent:
            self.get_logger().info(f'Successfully set mode to {mode}')
        else:
            self.get_logger().info(f'Failed to set mode to {mode}')

    def move_to_depth(self, goal_handle, target_depth):
        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = 'map'
        pose.pose.position.z = target_depth
        pose.pose.orientation.w = 1.0

        start_time = self.get_clock().now()
        max_duration = Duration(seconds=10.0)

        while (self.get_clock().now() - start_time) < max_duration:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled during depth movement')
                goal_handle.canceled()
                return False

            pose.header.stamp = self.get_clock().now().to_msg()
            self.position_publisher.publish(pose)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f'Moved to depth: {target_depth} meters')
        return True

    def rotate_to_yaw(self, goal_handle, target_yaw, max_duration=10.0):
        """Rotate to a specific yaw angle and update self.current_yaw accordingly."""
        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = 'map'
        pose.pose.position.z = -2.0
        pose.pose.orientation.w = math.cos(target_yaw * 0.5)
        pose.pose.orientation.z = math.sin(target_yaw * 0.5)

        start_time = self.get_clock().now()
        max_duration = Duration(seconds=max_duration)
        while (self.get_clock().now() - start_time) < max_duration:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled during yaw rotation')
                goal_handle.canceled()
                return False

            pose.header.stamp = self.get_clock().now().to_msg()
            self.position_publisher.publish(pose)
            rclpy.spin_once(self, timeout_sec=0.1)

        # Update current_yaw to align forward movement with the rotated yaw
        self.current_yaw = target_yaw
        self.stabilize_after_action(duration=2.0)
        return True

    def move_forward(self, goal_handle, distance=10.0):
        """Move forward using odometry to measure the actual distance traveled."""
        velocity = Twist()
        speed = 2.0
        self.start_position = self.current_position  # Set the start position for distance calculation

        while self.get_traveled_distance() < distance and rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled during forward movement')
                goal_handle.canceled()
                return False

            # Set movement based on current_yaw
            velocity.linear.x = speed * math.cos(self.current_yaw)
            velocity.linear.y = speed * math.sin(self.current_yaw)
            self.velocity_publisher.publish(velocity)

            traveled = self.get_traveled_distance()
            self.get_logger().info(f'Moving forward: traveled {traveled:.2f} m of {distance} m at speed {speed} m/s')
            rclpy.spin_once(self, timeout_sec=0.1)

        self.stop_movement()
        self.get_logger().info('Stopping movement')
        self.stabilize_after_action(duration=2.0)
        return True

    def get_traveled_distance(self):
        if self.start_position and self.current_position:
            dx = self.current_position.x - self.start_position.x
            dy = self.current_position.y - self.start_position.y
            return math.sqrt(dx ** 2 + dy ** 2)
        return 0.0

    def stop_movement(self):
        self.velocity_publisher.publish(Twist())

    def stabilize_after_action(self, duration=2.0):
        self.get_logger().info('Stabilizing to minimize drift...')
        start_time = self.get_clock().now()
        max_duration = Duration(seconds=duration)
        while (self.get_clock().now() - start_time) < max_duration and rclpy.ok():
            self.velocity_publisher.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.1)

    def execute_snail_pattern(self, goal_handle, initial_side_length=2.0, increment=2.0, max_side_length=20.0):
        """Execute the snail pattern, with each segment aligned to the specified yaw angles."""
        current_side_length = initial_side_length
        yaw_angles = [0, -90, -180, -270]
        move_count = 0

        while current_side_length <= max_side_length and rclpy.ok():
            for yaw in yaw_angles:
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal canceled during snail pattern execution')
                    goal_handle.canceled()
                    return False

                # Rotate to the specified yaw angle
                if not self.rotate_to_yaw(goal_handle, math.radians(yaw), max_duration=10.0):
                    return False
                # Move forward in the direction of the current yaw
                if not self.move_forward(goal_handle, distance=current_side_length):
                    return False

                move_count += 1

                if move_count % 2 == 0:
                    current_side_length += increment
                    self.get_logger().info(f'Increasing side length to {current_side_length} m')
        return True

def main(args=None):
    rclpy.init(args=args)
    autonomous_rov_controller = AutonomousROVController()
    executor = MultiThreadedExecutor()
    executor.add_node(autonomous_rov_controller)
    executor.spin()

if __name__ == '__main__':
    main()
