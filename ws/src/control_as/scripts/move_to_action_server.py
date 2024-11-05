#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rlab_customized_ros_msg.action import MoveTo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MoveToActionServer(Node):
    def __init__(self):
        super().__init__('move_to_action_server')
        
        # Action Server
        self._action_server = ActionServer(
            self,
            MoveTo,
            'move_to',
            self.execute_callback
        )

        # Publisher to move to target position
        self.position_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Subscriber to odometry for current position
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

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

    def odom_callback(self, msg):
        # Update current position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        target_pose = goal_handle.request.target_pose

        # Move towards target and give feedback
        rate = self.create_rate(1)  # Control the loop rate for feedback
        while True:
            # Calculate progress based on the current and target positions
            dx = target_pose.pose.position.x - self.current_x
            dy = target_pose.pose.position.y - self.current_y
            dz = target_pose.pose.position.z - self.current_z
            distance = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
            progress = max(0.0, min(1.0, 1 - distance / 10.0)) * 100  # Assume 10 meters max for full progress

            # Send feedback
            feedback_msg = MoveTo.Feedback()
            feedback_msg.progress = progress
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Progress: {progress:.2f}%')

            # Check if goal is reached
            if distance < 0.1:
                goal_handle.succeed()
                result = MoveTo.Result()
                result.success = True
                self.get_logger().info('Target position reached')
                return result

            # Publish the target pose to move towards
            self.position_publisher.publish(target_pose)
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    action_server = MoveToActionServer()
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

