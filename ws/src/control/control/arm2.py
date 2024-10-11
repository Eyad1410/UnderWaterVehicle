import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
import time

class AutonomousROVController(Node):
    def __init__(self):
        super().__init__('autonomous_rov_controller')
        self.position_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.arm_rov()
        self.set_mode('GUIDED')

    def arm_rov(self):
        # Create a service client to arm the ROV
        arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not arm_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        
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

    def move_to_position(self, x, y, z, duration=5.0):
        # Publish a position command to move the ROV
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        start_time = time.time()
        while time.time() - start_time < duration:
            self.position_publisher.publish(pose)
            self.get_logger().info(f'Moving to position: x={x}, y={y}, z={z}')
            time.sleep(0.1)  # Send the command at 10 Hz

    def execute_square_pattern(self, side_length, depth, duration=5.0):
        # Move in a square pattern at the specified depth
        self.get_logger().info('Starting square pattern navigation')

        # Define the four corners of the square relative to the starting point
        corners = [
            (0.0, 0.0, depth),
            (side_length, 0.0, depth),  # corners {1}
            (side_length, side_length, depth), # 2
            (0.0, side_length, depth), #3
            (0.0, 0.0, depth)  # Return to the starting point
        ]

        # Move to each corner sequentially
        for i in range(len(corners) - 1):
            x_start, y_start, z_start = corners[i]
            x_target, y_target, z_target = corners[i + 1]
            
            self.move_to_position(x_target, y_target, z_target, duration=duration)
            time.sleep(2)  # Pause for stability

        self.get_logger().info('Finished square pattern navigation')

def main(args=None):
    rclpy.init(args=args)

    # Initialize the autonomous ROV controller
    autonomous_controller = AutonomousROVController()
    
    # Move the ROV autonomously in a square pattern
    side_length = 2.0  # Length of the side of the square in meters
    depth = -2.0       # Depth to maintain during the square movement in meters
    autonomous_controller.execute_square_pattern(side_length, depth, duration=5.0)

    autonomous_controller.get_logger().info('Finished autonomous movement')

    # Shutdown the node
    autonomous_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
