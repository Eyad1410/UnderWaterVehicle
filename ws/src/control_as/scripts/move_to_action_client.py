#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rlab_customized_ros_msg.action import MoveTo
from geometry_msgs.msg import PoseStamped

class MoveToActionClient(Node):
    def __init__(self):
        super().__init__('move_to_action_client')
        
        # Action client for MoveTo action
        self._action_client = ActionClient(self, MoveTo, 'move_to')

    def send_goal(self, x, y, z):
        goal_msg = MoveTo.Goal()
        goal_msg.target_pose = PoseStamped()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.pose.position.x = x
        goal_msg.target_pose.pose.position.y = y
        goal_msg.target_pose.pose.position.z = z
        goal_msg.target_pose.pose.orientation.w = 1.0

        # Send goal
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        progress = feedback_msg.feedback.progress
        self.get_logger().info(f'Feedback: {progress:.2f}%')

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Goal reached successfully')
        else:
            self.get_logger().info('Goal failed')

def main(args=None):
    rclpy.init(args=args)
    action_client = MoveToActionClient()
    action_client.send_goal(5.0, 5.0, -2.0)  # Example target coordinates
    rclpy.spin(action_client)
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

