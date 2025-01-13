#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rlab_customized_ros_msg.action import SnailPattern
import time

class AutonomousROVClient(Node):
    def __init__(self):
        super().__init__('autonomous_rov_client')
        self._action_client = ActionClient(self, SnailPattern, 'snail_pattern')
        
        # Timer for regular checks, such as for cancellation
        self.timer = self.create_timer(1.0, self.timer_cb)
        self.counter = 0
        self._goal_handle = None  # Store the goal handle to use for canceling
        self.snail_pattern_completed = False  # Custom variable to track process completion

    def timer_cb(self):
        self.counter += 1
        # Cancel the goal if the timer has been called more than 1000 times
        if self.counter > 1000 and self._goal_handle is not None:
            self.get_logger().info('Cancelling goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal cancel failed')
        # Reset and send a new goal after cancellation
        self.reset_and_send_goal()

    def send_goal(self):
        self.get_logger().info('Waiting for SnailPattern action server...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return

        # Define the goal
        goal_msg = SnailPattern.Goal()
        goal_msg.initial_side_length = 2.0
        goal_msg.increment = 2.0
        goal_msg.max_side_length = 2.0

        self.get_logger().info(f'Sending SnailPattern goal: {goal_msg}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.reset_and_send_goal()  # Send a new goal if rejected
            return

        self.get_logger().info('Goal accepted')
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.progress * 100:.2f}% complete')

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Goal succeeded')
            self.snail_pattern_completed = True  # Set variable to True on success
        else:
            self.get_logger().info('Goal failed')
            self.snail_pattern_completed = False  # Optionally reset if needed

        # Reset and send a new goal after each result
        self.reset_and_send_goal()

    def reset_and_send_goal(self):
        """Reset the client state and send a new goal."""
        self._goal_handle = None
        self.counter = 0  # Reset counter
        if not self.snail_pattern_completed:
            self.send_goal()

def main(args=None):
    rclpy.init(args=args)
    rov_client = AutonomousROVClient()
    rov_client.send_goal()
    rclpy.spin(rov_client)

if __name__ == '__main__':
    main()

