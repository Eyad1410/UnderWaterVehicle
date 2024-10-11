import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.srv import SetMode, CommandBool
from std_srvs.srv import Trigger
import time
import math

class AutonomousROVServer(Node):
    def __init__(self):
        super().__init__('autonomous_rov_server')
        self.position_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.arm_rov()
        self.set_mode('GUIDED')
        self.current_yaw = 0.0  # Keep track of the current yaw angle
        self.target_yaw = 0.0  # Keep track of the intended yaw angle

        # Create services to receive commands
        self.create_service(Trigger, 'execute_snail_pattern', self.handle_execute_snail_pattern)
        self.create_service(Trigger, 'move_forward', self.handle_move_forward)

    def arm_rov(self):
        arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not arm_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        
        request = CommandBool.Request()
        request.value = True

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
        
        request = SetMode.Request()
        request.custom_mode = mode

        future = mode_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info(f'Successfully set mode to {mode}')
        else:
            self.get_logger().error(f'Failed to set mode to {mode}')

    def rotate_to_yaw(self, target_yaw, max_duration=10.0):
        self.get_logger().info(f'Rotating to yaw: {math.degrees(target_yaw)} degrees')
        self.target_yaw = target_yaw
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.z = -2.0  # Maintain depth at 2 meters

        start_time = time.time()
        while time.time() - start_time < max_duration:
            cy = math.cos(self.target_yaw * 0.5)
            sy = math.sin(self.target_yaw * 0.5)
            pose.pose.orientation.w = cy
            pose.pose.orientation.z = sy
            pose.header.stamp = self.get_clock().now().to_msg()
            self.position_publisher.publish(pose)
            time.sleep(0.05)

        # Update current yaw after rotation completes
        self.current_yaw = self.target_yaw
        self.stop_and_stabilize()  # Stabilize after rotation to minimize drift

    def move_forward(self, distance=10.0):
        velocity = Twist()
        speed = 0.5
        velocity.linear.x = speed * math.cos(self.current_yaw)
        velocity.linear.y = speed * math.sin(self.current_yaw)
        traveled_distance = 0.0
        last_time = time.time()

        while traveled_distance < distance and rclpy.ok():
            current_time = time.time()
            elapsed_time = current_time - last_time
            traveled_distance += speed * elapsed_time
            self.velocity_publisher.publish(velocity)
            last_time = current_time
            time.sleep(0.01)  # Increase frequency of updates for finer control

        self.stop_and_stabilize()  # Stabilize after movement to ensure accuracy
        self.maintain_position(duration=1.0)  # Minor stabilization to reduce drift after each forward movement

    def stop_movement(self):
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        self.velocity_publisher.publish(velocity)

    def execute_snail_pattern(self, initial_side_length=2.0, increment=2.0, max_side_length=10.0):
        current_side_length = initial_side_length
        yaw_angles = [0, -90, -180, -270]
        move_count = 0

        while current_side_length <= max_side_length and rclpy.ok():
            for yaw in yaw_angles:
                target_yaw = math.radians(yaw)
                self.rotate_to_yaw(target_yaw)
                self.stop_and_stabilize()  # Ensure no movement before counting distance
                self.move_forward(current_side_length)
                move_count += 1

                # Increment the side length after completing two sides
                if move_count % 2 == 0:
                    current_side_length += increment
                    if current_side_length > max_side_length:
                        break

    def handle_execute_snail_pattern(self, request, response):
        self.get_logger().info('Received request to execute snail pattern')
        self.execute_snail_pattern()
        response.success = True
        response.message = 'Snail pattern executed successfully'
        return response

    def handle_move_forward(self, request, response):
        self.get_logger().info('Received request to move forward')
        self.move_forward(distance=10.0)  # Example: move forward 10 meters
        response.success = True
        response.message = 'Move forward executed successfully'
        return response

    def maintain_position(self, duration=5.0):
        # Function to maintain position to reduce drift
        velocity = Twist()
        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            # Apply minimal corrections to counteract drift
            velocity.linear.x = 0.0
            velocity.linear.y = 0.0
            velocity.linear.z = 0.0
            velocity.angular.z = 0.0
            self.velocity_publisher.publish(velocity)
            time.sleep(0.05)

    def stop_and_stabilize(self):
        # Stop movement and stabilize to prevent drift
        self.stop_movement()
        self.maintain_position(duration=2.5)  # Increase stabilization duration to further reduce drift


def main(args=None):
    rclpy.init(args=args)
    server = AutonomousROVServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
