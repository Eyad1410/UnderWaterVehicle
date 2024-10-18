import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.srv import SetMode, CommandBool
from rlab_customized_ros_msg.action import MoveToPose
import math
import time

class AutonomousROVController(Node):
    def __init__(self):
        super().__init__('autonomous_rov_controller')
        self.position_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # Action server for MoveToPose
        self._action_server = ActionServer(
            self,
            MoveToPose,
            'move_to_pose',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.arm_rov()
        self.set_mode('GUIDED')
        self.current_yaw = 0.0  # Keep track of the current yaw angle
        self.target_yaw = 0.0  # Keep track of the intended yaw angle to ensure alignment

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received request to cancel goal')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        target_pose = goal_handle.request.target_pose
        self.get_logger().info(f"Moving to target pose: {target_pose}")

        feedback_msg = MoveToPose.Feedback()
        success = True

        # Simulate moving to target depth and snail pattern
        try:
            # Step 1: Move to target depth
            self.move_to_depth(target_pose.pose.position.z)
            feedback_msg.progress = 0.3
            goal_handle.publish_feedback(feedback_msg)

            # Step 2: Execute snail pattern
            self.execute_snail_pattern()
            feedback_msg.progress = 1.0
            goal_handle.publish_feedback(feedback_msg)
        except Exception as e:
            self.get_logger().error(f'Error during execution: {str(e)}')
            success = False

        # Check for goal cancellation
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            result = MoveToPose.Result()
            result.success = False
            return result

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

    def arm_rov(self):
        # Create a service client to arm the ROV
        arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not arm_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service... Please ensure the ROV is properly connected and powered on.')
        
        request = CommandBool.Request()
        request.value = True  # Set to False if you want to disarm

        future = arm_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('ROV armed successfully')
        else:
            self.get_logger().info('Failed to arm the ROV')

    def set_mode(self, mode):
        # Create a service client to set the mode of the ROV
        mode_service = self.create_client(SetMode, '/mavros/set_mode')
        while not mode_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')
        
        request = SetMode.Request()
        request.custom_mode = mode

        future = mode_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().mode_sent:
            self.get_logger().info(f'Successfully set mode to {mode}')
        else:
            self.get_logger().info(f'Failed to set mode to {mode}')

    def move_to_depth(self, target_depth):
        # Move to the desired depth by publishing the target pose
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.z = target_depth  # Maintain depth at target depth
        pose.pose.orientation.w = 1.0

        start_time = time.time()
        max_duration = 10.0
        while time.time() - start_time < max_duration:
            pose.header.stamp = self.get_clock().now().to_msg()
            self.position_publisher.publish(pose)
            time.sleep(0.1)  # Publish at 10 Hz

        self.get_logger().info(f'Moved to depth: {target_depth} meters')

    def rotate_to_yaw(self, target_yaw, max_duration=10.0):
        # Set target yaw and create feedback loop to reach it accurately
        self.get_logger().info(f'Rotating to yaw: {math.degrees(target_yaw)} degrees')
        self.target_yaw = target_yaw
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.z = -2.0  # Maintain depth at 2 meters

        start_time = time.time()
        while time.time() - start_time < max_duration:
            # Convert current yaw to quaternion
            cy = math.cos(self.target_yaw * 0.5)
            sy = math.sin(self.target_yaw * 0.5)
            pose.pose.orientation.w = cy
            pose.pose.orientation.z = sy
            pose.header.stamp = self.get_clock().now().to_msg()
            self.position_publisher.publish(pose)
            time.sleep(0.05)

        # Update current yaw after rotation completes
        self.current_yaw = self.target_yaw
        self.stabilize_after_action(duration=3.0)  # Stabilize after rotation to minimize drift

    def move_forward(self, distance=10.0):
        # Publish a velocity command to move the ROV forward in the direction of the current yaw
        velocity = Twist()
        speed = 0.3  # Reduced speed for more stability
        traveled_distance = 0.0
        last_time = time.time()

        while traveled_distance < distance and rclpy.ok():
            current_time = time.time()
            elapsed_time = current_time - last_time
            traveled_distance += speed * elapsed_time

            velocity.linear.x = speed * math.cos(self.target_yaw)  # Set speed in the x direction based on target yaw
            velocity.linear.y = speed * math.sin(self.target_yaw)  # Set speed in the y direction based on target yaw
            velocity.linear.z = 0.0  # Maintain depth
            self.velocity_publisher.publish(velocity)

            self.get_logger().info(f'Moving forward: traveled {traveled_distance:.2f} m of {distance} m at speed {speed} m/s')
            last_time = current_time
            time.sleep(0.1)

        # Stop the ROV after moving forward
        self.stop_movement()
        self.get_logger().info('Stopping movement')
        self.stabilize_after_action(duration=3.0)

    def stop_movement(self):
        # Publish a zero velocity command to stop all movement
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        self.velocity_publisher.publish(velocity)

    def stabilize_after_action(self, duration=3.0):
        # Function to maintain position to reduce drift after an action
        self.get_logger().info('Stabilizing to minimize drift...')
        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            velocity = Twist()
            velocity.linear.x = 0.0
            velocity.linear.y = 0.0
            velocity.linear.z = 0.0
            velocity.angular.x = 0.0
            velocity.angular.y = 0.0
            velocity.angular.z = 0.0
            self.velocity_publisher.publish(velocity)
            time.sleep(0.1)

    def execute_snail_pattern(self, initial_side_length=2.0, increment=2.0, max_side_length=10.0):
        current_side_length = initial_side_length
        yaw_angles = [0, -90, -180, -270]  # Yaw angles for each side of the square path
        move_count = 0

        while current_side_length <= max_side_length and rclpy.ok():
            for yaw in yaw_angles:
                # Rotate to the next yaw angle
                target_yaw = math.radians(yaw)
                self.rotate_to_yaw(target_yaw, max_duration=10.0)
                self.stabilize_after_action(duration=2.0)  # Ensure no movement before counting distance
                self.move_forward(distance=current_side_length)
                move_count += 1

                # Increment the side length after completing two sides
                if move_count % 2 == 0:
                    current_side_length += increment
                    self.get_logger().info(f'Increasing side length to {current_side_length} m')
                    if current_side_length > max_side_length:
                        break

def main(args=None):
    rclpy.init(args=args)
    autonomous_rov_controller = AutonomousROVController()

    try:
        rclpy.spin(autonomous_rov_controller)
    except KeyboardInterrupt:
        pass

    autonomous_rov_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

