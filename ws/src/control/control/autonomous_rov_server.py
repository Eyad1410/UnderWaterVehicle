import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.srv import SetMode, CommandBool
from rclpy.node import Node
from rlab_customized_ros_msg.action import MoveToPose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import math
from rclpy.duration import Duration

class AutonomousROVController(Node):
    def __init__(self):
        super().__init__('autonomous_rov_controller')

        self._action_server = ActionServer(
            self,
            MoveToPose,
            'move_to_pose',
            callback_group=ReentrantCallbackGroup(),
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.position_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        self.arm_rov()
        self.set_mode('GUIDED')
        self.current_yaw = 0.0  # Keep track of the current yaw angle
        self.target_yaw = 0.0  # Keep track of the intended yaw angle to ensure alignment

        self.previous_goal_handle = None
        self.current_goal_handle = None
        self.abort_previous_goal = False

        self.get_logger().info('autonomous_rov_controller is ready')

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        self.goal = goal_request

        if self.current_goal_handle and not self.previous_goal_handle:
            if self.current_goal_handle.is_active:
                self.get_logger().warn('Set the abort flag for previous_goal_handle')
                self.abort_previous_goal = True
                self.previous_goal_handle = self.current_goal_handle

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Start action move_to_pose')
        target_pose = goal_handle.request.target_pose
        self.get_logger().info(f'Moving to target pose: {target_pose}')

        feedback_msg = MoveToPose.Feedback()
        success = True

        self.current_goal_handle = goal_handle
        time.sleep(0.3)

        try:
            # Step 1: Move to target depth
            if not self.move_to_depth(goal_handle, target_pose.pose.position.z):
                return self.canceled_result()
            feedback_msg.progress = 0.3
            if not goal_handle.is_cancel_requested:
                goal_handle.publish_feedback(feedback_msg)

            # Step 2: Execute snail pattern
            if not self.execute_snail_pattern(goal_handle):
                return self.canceled_result()
            feedback_msg.progress = 1.0
            if not goal_handle.is_cancel_requested:
                goal_handle.publish_feedback(feedback_msg)
        except Exception as e:
            self.get_logger().error(f'Error during execution: {str(e)}')
            success = False

        # Final cancellation check
        if goal_handle.is_cancel_requested:
            self.get_logger().info('Goal canceled at the end of execution')
            goal_handle.canceled()
            return self.canceled_result()

        # Goal succeeded or failed
        if success:
            goal_handle.succeed()
            self.get_logger().info('Goal succeeded')
        else:
            goal_handle.abort()
            self.get_logger().info('Goal failed to reach target')

        result = MoveToPose.Result()
        result.success = success
        return result

    def canceled_result(self):
        return MoveToPose.Result(success=False)

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
        pose = PoseStamped(header={'frame_id': 'map'}, pose={'position': {'z': target_depth}, 'orientation': {'w': 1.0}})
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
        self.get_logger().info(f'Rotating to yaw: {math.degrees(target_yaw)} degrees')
        self.target_yaw = target_yaw
        pose = PoseStamped(header={'frame_id': 'map'}, pose={'position': {'z': -2.0}, 'orientation': {'w': math.cos(target_yaw * 0.5), 'z': math.sin(target_yaw * 0.5)}})

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

        self.current_yaw = self.target_yaw
        self.stabilize_after_action(duration=2.0)
        return True

    def move_forward(self, goal_handle, distance=10.0):
        velocity = Twist()
        speed = 0.3
        traveled_distance = 0.0
        last_time = self.get_clock().now()

        while traveled_distance < distance and rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled during forward movement')
                goal_handle.canceled()
                return False

            current_time = self.get_clock().now()
            elapsed_time = (current_time - last_time).nanoseconds / 1e9
            traveled_distance += speed * elapsed_time

            velocity.linear.x = speed * math.cos(self.target_yaw)
            velocity.linear.y = speed * math.sin(self.target_yaw)
            self.velocity_publisher.publish(velocity)

            self.get_logger().info(f'Moving forward: traveled {traveled_distance:.2f} m of {distance} m at speed {speed} m/s')
            last_time = current_time
            rclpy.spin_once(self, timeout_sec=0.1)

        self.stop_movement()
        self.get_logger().info('Stopping movement')
        self.stabilize_after_action(duration=2.0)
        return True

    def stop_movement(self):
        self.velocity_publisher.publish(Twist())

    def stabilize_after_action(self, duration=2.0):
        self.get_logger().info('Stabilizing to minimize drift...')
        start_time = self.get_clock().now()
        max_duration = Duration(seconds=duration)
        while (self.get_clock().now() - start_time) < max_duration and rclpy.ok():
            self.velocity_publisher.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.1)

    def execute_snail_pattern(self, goal_handle, initial_side_length=2.0, increment=2.0, max_side_length=10.0):
        current_side_length = initial_side_length
        yaw_angles = [0, -90, -180, -270]
        move_count = 0

        while current_side_length <= max_side_length and rclpy.ok():
            for yaw in yaw_angles:
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal canceled during snail pattern execution')
                    goal_handle.canceled()
                    return False

                if not self.rotate_to_yaw(goal_handle, math.radians(yaw), max_duration=10.0):
                    return False
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
