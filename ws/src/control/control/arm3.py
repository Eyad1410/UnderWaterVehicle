import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.srv import SetMode, CommandBool
import time
import math

class AutonomousROVController(Node):
    def __init__(self):
        super().__init__('autonomous_rov_controller')
        self.position_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.arm_rov()
        self.set_mode('GUIDED')
        self.current_yaw = 0.0  # Keep track of the current yaw angle

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

    def rotate_to_yaw(self, target_yaw, max_duration=10.0):
        # Set target yaw and create feedback loop to reach it accurately
        self.current_yaw = target_yaw

        # Publish a pose command to rotate the ROV to the updated yaw angle
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = -2.0  # Maintain depth at 2 meters

        start_time = time.time()
        while time.time() - start_time < max_duration:
            # Convert current yaw to quaternion
            cy = math.cos(self.current_yaw * 0.5)
            sy = math.sin(self.current_yaw * 0.5)

            pose.pose.orientation.w = cy
            pose.pose.orientation.z = sy

            pose.header.stamp = self.get_clock().now().to_msg()
            self.position_publisher.publish(pose)
            self.get_logger().info(f'Rotating to yaw: {math.degrees(self.current_yaw)} degrees')
            time.sleep(0.1)  # Send the command at 10 Hz

        # Stop any residual rotation
        self.stop_movement()

        # Pause to stabilize after rotation
        self.get_logger().info('Stabilizing after rotation...')
        time.sleep(2)  # Reduced stabilization time for quicker response

    def move_forward(self, distance=10.0):
        # Publish a velocity command to move the ROV forward in the direction of the current yaw
        velocity = Twist()
        speed = 0.5  # Reduced speed for more accurate movement
        velocity.linear.x = speed * math.cos(self.current_yaw)  # Set speed in the x direction based on yaw
        velocity.linear.y = speed * math.sin(self.current_yaw)  # Set speed in the y direction based on yaw
        velocity.linear.z = 0.0  # Maintain depth

        traveled_distance = 0.0
        last_time = time.time()

        while traveled_distance < distance and rclpy.ok():
            current_time = time.time()
            elapsed_time = current_time - last_time
            traveled_distance += speed * elapsed_time
            self.velocity_publisher.publish(velocity)
            self.get_logger().info(f'Moving forward: traveled {traveled_distance:.2f} m of {distance} m at speed {speed} m/s')
            last_time = current_time
            time.sleep(0.1)  # Send the command at 10 Hz

        # Stop the ROV after moving forward
        self.stop_movement()
        self.get_logger().info('Stopping movement')

    def stop_movement(self):
        # Publish a zero velocity command to stop all movement
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        self.velocity_publisher.publish(velocity)

    def execute_snail_pattern(self, initial_side_length=10.0, decrement=2.0):
        current_side_length = initial_side_length
        yaw_angles = [0, -90, -180, -270]  # Yaw angles for each side of the square path
        move_count = 0

        while current_side_length > 0 and rclpy.ok():
            for yaw in yaw_angles:
                # Rotate to the next yaw angle
                target_yaw = math.radians(yaw)
                self.rotate_to_yaw(target_yaw, max_duration=10.0)
                
                # Move forward in the direction of the current yaw
                self.move_forward(distance=current_side_length)
                
                move_count += 1
                # Decrease the side length after completing three moves to create the snail pattern
                if move_count % 3 == 0:
                    current_side_length -= decrement
                    self.get_logger().info(f'Decreasing side length to {current_side_length} m')
                    if current_side_length <= 0:
                        break


def main(args=None):
    rclpy.init(args=args)
    autonomous_controller = AutonomousROVController()
    
    # Execute the snail pattern movement
    autonomous_controller.execute_snail_pattern()

    autonomous_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
